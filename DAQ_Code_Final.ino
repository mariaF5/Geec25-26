/*
* Author: Conor Farrell & Breandán Gillanders
* Edited by Chrislyn Forde, Isra Yousif, Lexine Estremera
* Date: 11/04/2025
*
* This is the code used in the SEM competition
*
* UPDATE: readADC task is renamed sensorTask as speed sensor is included
* UPDATE: speed sensor calibration added
*/


#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

//SD Card Variables
#define REASSIGN_PINS
int sck = 48;
int miso = 47;
int mosi = 38;
int cs = 21;
String filename = "/default.txt";  // pretty sure the part before the dot has to be less than or equal to 8 chars (DOS8.3 naming)
String prefix = "/ILTESTr";

Adafruit_ADS1115 ads;

//GPS Variables
int PWRPin = D8;  //This pin needs to be soldered to the PCB
char sim_response[512];
String nmeaSentence;
int ledState = LOW;
const int ledPin = D9;
String response, values;
int startIndex, endIndex = -1;
String GPSSpeed;

//Speed Sensor
const int SPEEDPIN = D2;  // sensor outputs, connected to D2 in new PCB
//volatile double speed;
float wheelCircumference = 2.5;            // Wheel circumference in meters (adjust for your wheel)
const int spokes = 6;                      // Number of spokes per disc brake rotation
const unsigned long timeout = 3000;        // 2-second timeout for zero speed detection
volatile unsigned long lastPulseTime = 0;  // Time of last pulse
volatile unsigned long rotationTime = 0;   // Time for one full disc brake rotation
volatile float wheelSpeed = 0.0;           // Speed in km/h


//CAN Bus
typedef union {
  float val;
  unsigned char bytes[4];
} FLOATUNION_t;
FLOATUNION_t canBuff;
#define CANADDRESS 0x25

#define configTICK_RATE_HZ 500
struct data_t {
  volatile float speed;                     
  float iBatt, iMot, vBat, vMot, lat, lon;  
  String GPS;
};

data_t data = { 0, 0, 0, 0, 0, 0, 0, "" };
static TaskHandle_t I2CactiveTask = NULL;  // Activity Semaphore for I2C bus


//speed sensor interrupt
volatile int isrCounter = 0;
void IRAM_ATTR countPulse() {
  static unsigned long lastDebounceTime = 0;
  static unsigned long lastFullRotationTime = 0;
  static int spokeCount = 0;
  const unsigned long debounceDelay = 40;  // 30 ms debounce, allows for speed of up to 40 km/hr to be read by the speed sensor
  unsigned long now = millis();
  if (now - lastDebounceTime >= debounceDelay) {
    lastDebounceTime = now;
    isrCounter++;
    if (++spokeCount >= spokes) {
      rotationTime = now - lastFullRotationTime;
      lastPulseTime = now;
      lastFullRotationTime = now;
      spokeCount = 0;
    }
  }
}

void setup() {


  Serial0.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  pinMode(SPEEDPIN, INPUT_PULLUP);  // Pull-up enabled for clean signal
  attachInterrupt(digitalPinToInterrupt(SPEEDPIN), countPulse, RISING);
  //data.speed = 0.0;

  pinMode(ledPin, OUTPUT);
  pinMode(PWRPin, OUTPUT);

  lastPulseTime = millis();  // initialize to avoid false timeout

  adcSetup();

  sdSetup();

  setNextFileName(prefix);  // DOS8.3 naming, must instert 5 char prefix rn

  // attachInterrupt(digitalPinToInterrupt(SPEEDPIN), currSpeed, FALLING);

  delay(1000);
  xTaskCreate(
    sensorTask,   /* Task function. */
    "sensorTask", /* String with name of task. */
    10000,     /* Stack size in bytes. */
    NULL,      /* Parameter passed as input of the task */
    1,         /* Priority of the task. */
    NULL);     /* Task handle. */

  xTaskCreate(
    canTask,
    "canTask",
    10000,
    NULL,
    1,
    NULL);
  gpsSetup();

  xTaskCreate(
    pollGPSTask,   /* Task function. */
    "pollGPSTask", /* String with name of task. */
    10000,         /* Stack size in bytes. */
    NULL,          /* Parameter passed as input of the task */
    1,             /* Priority of the task. */
    NULL);         /* Task handle. */
}

void loop() {
  //NOTHING
}


void adcSetup() {
  ads.setGain(GAIN_TWOTHIRDS);
  ads.setDataRate(RATE_ADS1115_860SPS);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    digitalWrite(ledPin, HIGH);
    delay(2000);
    digitalWrite(ledPin, LOW);
    while (1)
      ;  //TODO ADD ERROR CODE LED
  }
}

//RTOS Tasks
//readADC is now called sensor task as wheel speed sensor is included
void sensorTask(void *parameter) {
  for (;;) {
    if (I2CactiveTask == NULL) {
      I2CactiveTask = xTaskGetCurrentTaskHandle();

      // Disable interrupts briefly to safely read shared variables, filtering
      noInterrupts();
      unsigned long currentRotationTime = rotationTime;
      unsigned long currentLastPulseTime = lastPulseTime;
      int currentISRCount = isrCounter;
      interrupts();
      unsigned long currentTime = millis();
      if (currentRotationTime >= 100 && currentRotationTime <= 10000 && (currentTime - currentLastPulseTime) < timeout) {
        float timeSeconds = currentRotationTime / 1000.0;
        wheelSpeed = (wheelCircumference / timeSeconds) * 3.6;  // m/s to km/h
      } else {
        wheelSpeed = 0.0;
      }

      //data.speed = wheelSpeed;
                                        

      //read adc values
      int16_t adc0, adc1, adc2, adc3;
      float v1, v2, v3, v4;

      adc0 = ads.readADC_SingleEnded(0);
      adc1 = ads.readADC_SingleEnded(1);
      adc2 = ads.readADC_SingleEnded(2);
      adc3 = ads.readADC_SingleEnded(3);

      I2CactiveTask = NULL;

      data.iBatt = 10 * (ads.computeVolts(adc0) - 0.493);
      data.iMot = 10 * (ads.computeVolts(adc1) - 0.493);
      data.vMot = 7.51 * (ads.computeVolts(adc2));
      data.vBat = 9.01 * (ads.computeVolts(adc3));

      String iB = String(data.iBatt, 3);
      String iM = String(data.iMot, 3);
      String vB = String(data.vBat, 3);
      String vM = String(data.vMot, 3);
      String message = String(millis()) + "," + iB + "," + iM + "," + vB + "," + vM + "," + wheelSpeed + "," + data.GPS + "\n";

      //Serial1.println(message);
      appendFile(SD, filename, message);


      vTaskDelay(pdMS_TO_TICKS((193)));
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  // Serial1.println("Ending task 1");
  vTaskDelete(NULL);
}

void SaveToSDTask(void *parameter) {
  for (;;) {
    // Serial1.printf("%f,%f,%f,%f \n", t.iBatt, t.iMot,t.vBat, t.vMot);
    String iB = String(data.iBatt, 3);
    String iM = String(data.iMot, 3);
    String vB = String(data.vBat, 3);
    String vM = String(data.vMot, 3);
    String message = String(millis()) + "," + iB + "," + iM + "," + vB + "," + vM + "," + String(data.speed, 1) + "," + data.GPS + "\n";
    // char messageArray[64] = message;
    appendFile(SD, filename, message);

    vTaskDelete(NULL);  //Task deletes itself
  }
}

void pollGPSTask(void *parameter) {
  for (;;) {
    response = send_at("AT+CGNSINF");
    startIndex = response.indexOf(",") - 1;
    endIndex = response.indexOf(",,");
    if (startIndex + 1 != endIndex) {
      data.GPS = response.substring(startIndex, endIndex);

      // Parse the GPS string
      int comma1 = data.GPS.indexOf(',');
      int comma2 = data.GPS.indexOf(',', comma1 + 1);
      int comma3 = data.GPS.indexOf(',', comma2 + 1);
      int comma4 = data.GPS.indexOf(',', comma3 + 1);
      int comma5 = data.GPS.indexOf(',', comma4 + 1);
      int comma6 = data.GPS.indexOf(',', comma5 + 1);

      if (comma6 != -1) {
        data.lat = data.GPS.substring(comma3 + 1, comma4).toFloat();
        data.lon = data.GPS.substring(comma4 + 1, comma5).toFloat();
      }

      GPSSpeed = data.GPS.substring(data.GPS.lastIndexOf(",") + 1, data.GPS.length());
      data.speed = GPSSpeed.toFloat();
    }

    // Serial1.println(t.GPS);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//TODO Add Method to take GPS speed
void canTask(void *parameter) {
  for (;;) {
    if (I2CactiveTask == NULL) {
      I2CactiveTask = xTaskGetCurrentTaskHandle();
      canBuff.val = data.speed;// wheel speed
      sendMsgBuf(0x01, 0, 0, 4, canBuff.bytes);
      delay(5);
      canBuff.val = data.iBatt;// battery current
      sendMsgBuf(0x02, 0, 0, 4, canBuff.bytes);
      delay(5);
      canBuff.val = data.vBat;// battery voltage
      sendMsgBuf(0x03, 0, 0, 4, canBuff.bytes);
      delay(5);
      canBuff.val = data.vMot;// motor voltage
      sendMsgBuf(0x06, 0, 0, 4, canBuff.bytes);
      delay(5);
      canBuff.val = data.iMot;// motor current
      sendMsgBuf(0x04, 0, 0, 4, canBuff.bytes);
      delay(5);
      unsigned long timestamp = millis(); //millis time
      sendMsgBuf(0x05, 0, 0, 4, (unsigned char *)&timestamp);
      delay(5);
      canBuff.val = data.lat; // latitude
      sendMsgBuf(0x07, 0, 0, 4, canBuff.bytes);
      delay(5);
      canBuff.val = data.lon; // longitude
      sendMsgBuf(0x08, 0, 0, 4, canBuff.bytes);
      I2CactiveTask = NULL;
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(8));  //Time ADC task takes to run
    }
  }
}


//SD Functions
void sdSetup() {

#ifdef REASSIGN_PINS
  SPI.begin(sck, miso, mosi, cs);
#endif
  if (!SD.begin(cs)) {  //Change to this function to manually change CS pin
    // Serial1.println("Card Mount Failed");
    while (true) {
      digitalWrite(ledPin, HIGH);
      delay(2000);
      digitalWrite(ledPin, LOW);
      delay(3000);
    };
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    // Serial1.println("No SD card attached");
    return;
  }
}

String setNextFileName(String prefix) {
  String next = "";
  int highest = -1;
  int prefixLength = prefix.length() - 1;
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      break;
    }
    String name = entry.name();
    if (name.startsWith(prefix.substring(1, prefixLength + 1)) && name.endsWith(".txt")) {
      String numberStr = name.substring(prefixLength, prefixLength + 3);  // Extract XXX part
      int number = numberStr.toInt();
      if (number > highest) {
        highest = number;
      }
    }
    entry.close();
  }
  highest++;
  char buffer[4];                    // Buffer to hold the padded number
  sprintf(buffer, "%03d", highest);  // Format the number with leading zeros
  next = prefix + String(buffer) + ".txt";
  filename = next;
  return next;
}


void appendFile(fs::FS &fs, String path, String message) {
  // Serial1.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial1.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial1.println("Message appended");
  } else {
    // Serial1.println("Append failed");
  }
  file.close();
}

//GPS Functions

void gpsSetup() {
  // Toggle power
  digitalWrite(PWRPin, HIGH);
  delay(1000);
  digitalWrite(PWRPin, LOW);
  delay(1000);

  send_at("AT+CGNSCOLD");

  // Serial1.println("Sending AT+CGNSHOT:");
  // Serial1.println(send_at("AT+CGNSHOT"));

  bool gpsInitialised = false;
  while (gpsInitialised == false) {
    gpsInitialised = validResponse(send_at("AT+CGNSINF"));
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    // Serial1.println("Checking");
    delay(500);
  }
  digitalWrite(ledPin, HIGH);
}

bool get_ok_response(String at_command) {
  bool ok = false;
  String response = send_at(at_command);
  int x = response.indexOf("OK");
  if (x != -1) {
    ok = true;
  }
  return ok;
}

bool validResponse(String response) {
  // Serial1.println(response);
  if (response.indexOf("+CGNSINF: ") != -1) {
    int start = response.indexOf(",") - 1;
    int end = response.lastIndexOf(",");
    String values = response.substring(start, end);
    // Serial1.println(values);
    if (values.charAt(0) == '1') {
      if (values.charAt(2) == '1') {
        return true;
      }
    }
  }
  return false;
}

String send_at(String at_command) {
  Serial1.flush();
  Serial1.println(at_command);
  while (!Serial1.available()) {
    delay(1);  //No operation
  }

  int t = 0;
  char c = 0;

  sim_response[0] = 0;

  while (Serial1.available()) {
    c = Serial1.read();
    sim_response[t] = c;
    sim_response[++t] = 0;
  }

  return sim_response;
}

void sendMsgBuf(unsigned long id, byte ext, byte rtr, byte len, unsigned char *buf) {
  unsigned char dta[16];

  dta[0] = 0xff & (id >> 24);
  dta[1] = 0xff & (id >> 16);
  dta[2] = 0xff & (id >> 8);
  dta[3] = 0xff & (id >> 0);

  dta[4] = ext;
  dta[5] = rtr;

  dta[6] = len;

  for (int i = 0; i < len; i++) {
    dta[7 + i] = buf[i];
  }
  dta[15] = makeCheckSum(dta, 15);

  IIC_CAN_SetReg(0x30, 16, dta);
}

unsigned char makeCheckSum(unsigned char *dta, int len) {
  unsigned long sum = 0;
  for (int i = 0; i < len; i++) sum += dta[i];

  if (sum > 0xff) {
    sum = ~sum;
    sum += 1;
  }

  sum = sum & 0xff;
  return sum;
}

void IIC_CAN_SetReg(unsigned char __reg, unsigned char __len, unsigned char *__dta) {
  Wire.beginTransmission(CANADDRESS);
  Wire.write(__reg);
  for (int i = 0; i < __len; i++) {

    Wire.write(__dta[i]);
  }
  Wire.endTransmission();
}
