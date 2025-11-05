#include <Wire.h>
#include "Longan_I2C_CAN_Arduino.h" // communicates with CAN Bus module
#include "rgb_lcd.h"                 // LCD screen
#include <SoftwareSerial.h>          // serial monitor / Wio-E5

// LCD Variables
rgb_lcd lcd; 
// adjust to change LCD backlight colour
const int colorR = 0;
const int colorG = 50;
const int colorB = 0;

// Button / interrupts / timers
const byte interruptPin = 2; // lap button is on pin 2
volatile int  lapNumber = 0;
const int debounceTime = 250;
volatile unsigned long runStartTime   = 0;
volatile unsigned long lastPress      = 0;
volatile unsigned long lapStartTime   = 0;
volatile boolean running              = false;
volatile boolean restartTimer         = false;
volatile unsigned long firstPress     = 0;
volatile unsigned long resetTimerPressDelay = 1500;
volatile int  numPresses              = 0;
volatile boolean pressed              = false;

// CAN decode helpers 
typedef union {
  float val;
  byte  bytes[4];
} FLOATUNION_t;

FLOATUNION_t buff;
unsigned char  len = 0;
unsigned long  canId;
I2C_CAN        CAN(0x25);  // Set I2C Address

//  Wio-E5 
SoftwareSerial WioE5(5, 4); // RX (Arduino), TX (Arduino) -> TX, RX (Wio-E5)

// Telemetry snapshot 
struct Telemetry {
  float wheelSpeed = 0.0f;
  float motorCurr  = 0.0f;
  float motorVolt  = 0.0f;
  float battVolt   = 0.0f;
  float battCurr   = 0.0f;
  float lat        = 0.0f;
  float lon        = 0.0f;
  unsigned long timestamp = 0;

  // Timestamps (ms)
  unsigned long t_wheel=0, t_mcurr=0, t_mvolt=0, t_bvolt=0, t_bcurr=0, t_gps=0, t_stamp=0;
} T;

// Energy / misc
float power = 0.0f;
float energy = 0.0f;
unsigned long prevRecTime = 0;

// Schedulers 
const uint16_t LCD_PERIOD_MS      = 100;  // 10 Hz LCD refresh
const uint16_t LORA_MIN_PERIOD_MS = 200;  // <= 5 Hz LoRa TX
unsigned long lastLcdMs = 0;
unsigned long lastLoRaMs = 0;
unsigned long lastSentTimestamp = 0;

// LCD diff-caches
char prevSpeed[6]    = ""; 
char prevThr[4]      = "";  
char prevCurr[6]     = ""; 
char prevLap[4]      = "";  
char prevLapTime[6]  = "";  
char prevRunTime[6]  = "";  

bool layoutPromptShown = false;   // "Press lap button..."
bool layoutRunningDrawn = false;  // one-time labels while running

// Forward decls
void readMsg();
void writeToLCD();
bool shouldSendLoRa(unsigned long now);
void loraSend();
void lapButtonPress();
void buttonPressed();
void timerSetup();


// SETUP
void setup() {
  delay(200);
  // I2C faster for LCD and I2C-CAN
  Wire.begin();
  Wire.setClock(400000);

  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);

  Serial.begin(9600);
  Serial.println("Entered setup()");
  WioE5.begin(9600);  // per your requirement, unchanged

  Serial.println("begin to init can");
  while (CAN_OK != CAN.begin(CAN_500KBPS)) { // init can bus : baudrate = 500k
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");

  lcd.setCursor(0,0); lcd.print("Starting");
  delay(600);
  lcd.clear();

  // Initialize lap button interrupt pin and interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), lapButtonPress, FALLING);
  Serial.println("Finished setup");

  // Wio-E5 init (unchanged)
  Serial.println("Initializing Wio-E5...");
  delay(300);
  WioE5.print("AT+MODE=TEST\r\n");
  delay(300);
  // Non-blocking readback skipped to avoid jitter
  WioE5.print("AT+TEST=RFCFG,868,7,125,12,15,22,ON,OFF,OFF\r\n");
  delay(300);

  // Prime energy timer on first Vbatt reception
  prevRecTime = millis();
}

// LOOP
void loop() {
  if (pressed) {
    pressed = false;
    buttonPressed();
  }

  if (restartTimer) {
    restartTimer = false;
    lapNumber = 0;
    numPresses = 0;
    energy = 0;
    running = false;

    // reset LCD caches so next draw repaints
    prevSpeed[0]=prevThr[0]=prevCurr[0]=prevLap[0]=prevLapTime[0]=prevRunTime[0]='\0';
    layoutPromptShown   = false;
    layoutRunningDrawn  = false;
    lcd.clear();
  }

  // Read CAN once per loop and update snapshot
  readMsg();

  const unsigned long now = millis();

  // LCD update @ 10 Hz
  if (now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    writeToLCD();
  }

  // LoRa update only on new timestamp & ≤ 5 Hz
  if (shouldSendLoRa(now)) {
    loraSend();
  }
}

// CAN READ (single pass)
void readMsg() {
  const unsigned long now = millis();
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buff.bytes);
    canId = CAN.getCanId();

    switch (canId) {
      case 1: // GPS Speed
        T.wheelSpeed = buff.val;
        T.t_wheel = now;
        break;

      case 2: // Battery Current
        T.battCurr = buff.val;
        T.t_bcurr = now;
        break;

      case 3: { // Battery Voltage
        unsigned long currRecTime = now;
        T.battVolt = buff.val;
        T.t_bvolt = now;

        // Power/Energy calc (Wh); guard first-interval spike
        if (prevRecTime == 0) prevRecTime = currRecTime;
        power  = T.battCurr * T.battVolt; // W
        energy += power * (currRecTime - prevRecTime) / 3600000.0f; // Wh
        prevRecTime = currRecTime;
        break;
      }

      case 4: // Motor Current
        T.motorCurr = buff.val;
        T.t_mcurr = now;
        break;

      case 5: // Timestamp
        T.timestamp = *(unsigned long*)buff.bytes;
        T.t_stamp   = now;
        break;

      case 6: // Motor Voltage
        T.motorVolt = buff.val;
        T.t_mvolt = now;
        break;

      case 7: // Latitude
        T.lat = buff.val;
        T.t_gps = now;
        break;

      case 8: // Longitude
        T.lon = buff.val;
        T.t_gps = now;
        break;
    }
  }
}

// LCD RENDER (diff-only)
static void printAtIfChanged(uint8_t col, uint8_t row, const char* s, char* cache, size_t cacheLen) {
  if (strncmp(s, cache, cacheLen-1) != 0) {
    lcd.setCursor(col, row);
    lcd.print(s);
    strncpy(cache, s, cacheLen-1);
    cache[cacheLen-1] = '\0';
  }
}

static void drawRunningLabels() {
  if (layoutRunningDrawn) return;
  // Units/labels drawn once to reduce I2C traffic
  lcd.setCursor(4, 0); lcd.print("kph"); // speed units
  lcd.setCursor(9, 0); lcd.print("%");   // throttle unit right after 2 digits
  lcd.setCursor(15,0); lcd.print("A");   // current unit in last column
  lcd.setCursor(13,1); lcd.print("L");   // Lap label at col 13
  layoutRunningDrawn = true;
}

void writeToLCD() {
  if (!running) {
    if (!layoutPromptShown) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Press lap button");
      lcd.setCursor(0,1); lcd.print("to start timer");
      layoutPromptShown = true;
    }
    return;
  }

  // When we start running, ensure labels are drawn once
  drawRunningLabels();

  // 1) Speed "00.0" at (0,0); "kph" already printed at (4,0)
  {
    char s[8];
    float sp = T.wheelSpeed;
    if (sp < 0) sp = 0; if (sp > 999.9f) sp = 999.9f;
    // fixed width 4 incl decimal -> pad with zeros
    // e.g.,  "00.0"
    int whole = (int)sp;
    int tenths = (int)((sp - whole) * 10 + 0.5f);
    snprintf(s, sizeof(s), "%02d.%1d", whole%100, tenths%10);
    printAtIfChanged(0, 0, s, prevSpeed, sizeof(prevSpeed));
  }

  // 2) Throttle "00" (we place digits at (7,0), '%' is at (9,0))
  {
    char s[4];
    float thr = 0.0f;
    if (T.battVolt > 1.0f && isfinite(T.motorVolt)) {
      thr = (T.motorVolt / T.battVolt) * 100.0f;
    }
    if (thr < 0) thr = 0;
    if (thr > 99) thr = 99; // 2 digits
    snprintf(s, sizeof(s), "%02d", (int)(thr + 0.5f));
    printAtIfChanged(7, 0, s, prevThr, sizeof(prevThr));
  }

  // 3) Battery current "00.0" (with "A" already at (15,0))
  {
    char s[8];
    float ia = T.battCurr;
    if (!isfinite(ia)) ia = 0.0f;
    // Format to width 4 with one decimal; pad with zeros
    // We'll print at (11,0) and leave "A" at 15
    int whole = (int)fabs(ia);
    int tenths = (int)((fabs(ia) - whole) * 10 + 0.5f);
    // If negative, clamp to zero for driver display (optional)
    if (ia < 0) { whole = 0; tenths = 0; }
    snprintf(s, sizeof(s), "%02d.%1d", whole%100, tenths%10);
    printAtIfChanged(11, 0, s, prevCurr, sizeof(prevCurr));
  }

  const unsigned long now = millis();

  // 4) Lap time "MM:SS" at (0,1)
  {
    unsigned long lapMs = now - lapStartTime;
    unsigned int lsec = lapMs / 1000;
    unsigned int mm = lsec / 60;
    unsigned int ss = lsec % 60;
    char s[8];
    snprintf(s, sizeof(s), "%01u:%02u", mm, ss);
    printAtIfChanged(0, 1, s, prevLapTime, sizeof(prevLapTime));
  }

  // 5) Run time "MM:SS" at (6,1)
  {
    unsigned long runMs = now - runStartTime;
    unsigned int rsec = runMs / 1000;
    unsigned int mm = rsec / 60;
    unsigned int ss = rsec % 60;
    char s[8];
    snprintf(s, sizeof(s), "%01u:%02u", mm, ss);
    printAtIfChanged(6, 1, s, prevRunTime, sizeof(prevRunTime));
  }

  // 6) Lap number "L00" – we only update the digits (label 'L' is at 13)
  {
    char s[4];
    snprintf(s, sizeof(s), "%02d", lapNumber % 100);
    printAtIfChanged(14, 1, s, prevLap, sizeof(prevLap));
  }
}

// LoRa TX (only when fresh)
bool shouldSendLoRa(unsigned long now) {
  if (T.timestamp == 0) return false;                     // need data
  if (T.timestamp == lastSentTimestamp) return false;     // only new
  if (now - lastLoRaMs < LORA_MIN_PERIOD_MS) return false; // rate limit
  return true;
}

void loraSend() {
  lastLoRaMs = millis();
  lastSentTimestamp = T.timestamp;

  // Throttle calculation for payload
  float throttle = 0.0f;
  if (T.battVolt > 1.0f && isfinite(T.motorVolt)) {
    throttle = (T.motorVolt / T.battVolt) * 100.0f;
    if (throttle < 0) throttle = 0;
    if (throttle > 100) throttle = 100;
  }

  // Buffer payload with cutoff decimals
  char payload[128];
  // timestamp,Imot,Vmot,Ibatt,Vbatt,spd,lat,lon
  snprintf(payload, sizeof(payload), "%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.4f,%.4f",
           T.timestamp, T.motorCurr, T.motorVolt, T.battCurr, T.battVolt,
           T.wheelSpeed, T.lat, T.lon);

  // Send AT once; avoid blocking reads to keep latency low
  WioE5.print("AT+TEST=TXLRSTR,\"");
  WioE5.print(payload);
  WioE5.print("\"\r\n");

}

// Timers / Buttons / ISR
void timerSetup() {
  const unsigned long t = millis();
  runStartTime = t;
  lapStartTime = t;
  running = true;

  // Reset LCD layout flags so running labels draw
  layoutPromptShown  = false;
  layoutRunningDrawn = false;
  // Clear once on transition to running
  lcd.clear();
}

void lapButtonPress() {
  pressed = true; // do as little as possible in ISR
}

void buttonPressed() {
  const unsigned long now = millis();
  if (now - lastPress > (unsigned long)debounceTime) {
    lapNumber++;
    // Triple-press within resetTimerPressDelay -> restart overall timer
    if (now - firstPress < resetTimerPressDelay) {
      numPresses++;
      if (numPresses > 2) {
        restartTimer = true;
      }
    } else {
      firstPress = now;  // start new sequence
      numPresses = 1;
    }

    lastPress = now;
    lapStartTime = now;

    if (lapNumber == 1) {
      timerSetup();
    }
  }
}
