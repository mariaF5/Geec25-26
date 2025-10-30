#include <Wire.h>
#include "Longan_I2C_CAN_Arduino.h"
#include "rgb_lcd.h"

//Variables for LCD
rgb_lcd lcd;
const int colorR = 0;
const int colorG = 50; // NEED TO TUNE THIS IN FRANCE
const int colorB = 0;

//Variables for interrupt
const byte interruptPin = 3;
volatile int lapNumber = 0;
const int debounceTime = 250;
volatile long runStartTime = 0;
volatile long lastPress = 0;
volatile long lapStartTime = 0;
volatile boolean running = false;
volatile boolean restartTimer = false;
volatile long firstPress = 0;
volatile long resetTimerPressDelay = 1500;
volatile int numPresses = 0;

volatile boolean pressed = false;

typedef union{
  float val;
  byte bytes[4];
} FLOATUNION_t;

FLOATUNION_t buff;
float speedRx;
float currRx;
float voltRx;
float vMotRx;

I2C_CAN CAN(0x25);  // Set I2C Address
unsigned char len = 0;
unsigned long id;

// Other Variables
float power = 0;
String energyString, currString, speedString, throttleString = "";
float energy = 0;
float throttle = 0;
unsigned long currRecTime, prevRecTime = 0;
unsigned long currentTime = 0;
unsigned long elapsedTime = 0;
unsigned int seconds = 0;
unsigned int minutes = 0;

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);

  Serial.begin(9600);
  while(!Serial);

  Serial.println("begin to init can");
  while (CAN_OK != CAN.begin(CAN_500KBPS)) { // init can bus : baudrate = 500k
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");

  lcd.setCursor(0,0);
  lcd.print("Starting");
  delay(1000);
  lcd.clear();
  delay(100);


  // Initialize lap button interrupt pin and interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), lapButtonPress, FALLING);
  Serial.println("Finished setup");
  if (running){
    Serial.println("RUNNING IS TRUE");
  }
}

void loop(){
  if (pressed){
    pressed = false;
    buttonPressed();
  }
  if (restartTimer){
    restartTimer = false;
    lapNumber = 0;
    numPresses = 0;
    energy = 0;
    running = false;
  }

  writeToLCD();
  delay(100);
}

void readMsg(){
   while ( CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buff.bytes);
    id = CAN.getCanId();
    Serial.print("CAN ID: ");
    Serial.println(id);

    // DIFFERENT READING TYPES BASED ON CAN MESSAGE ID
    switch (CAN.getCanId()){
      // SPEED READING
      case 1:
        speedRx = buff.val;
        speedRx = speedRx*3.6;
        Serial.println(speedRx);
        break;
      // CURRENT READING
      case 2:
        currRx = buff.val;
        Serial.println(currRx);
        break;
      // VOLTAGE READING, CALCULATES POWER AND ENERGY
      case 3:
        currRecTime = millis();
        voltRx = buff.val;
        Serial.println(voltRx);
        power = currRx * voltRx;
        energy += power*(currRecTime-prevRecTime)/3600000; //Wh conversion
        Serial.println(energy);
        prevRecTime = currRecTime;
        break;
      case 4:
        vMotRx = buff.val;
        throttle = (vMotRx / voltRx) * 100;
        break;
    }
  }
}

/* LCD Format
00.0kphXX00%XX00
00:00X00:00XXL00
*/

/* LCD Format
00.0kph00%X00.0A
00:00X00:00XXL00
*/
void writeToLCD(){
  if (running){
    readMsg(); //CAN Bus code

    // lcd.clear();
    lcd.setCursor(0, 0);
    speedString = String(speedRx, 1);
    speedString = stringFormat(speedString, 4);
    lcd.print(speedString);
    lcd.print("kph");

    lcd.setCursor(7, 0);
    throttleString = String(throttle, 0);
    throttleString = stringFormat(throttleString, 2);
    lcd.print(throttleString);
    lcd.print("%");

    lcd.setCursor(11, 0);
    currString = String(currRx, 1);
    currString = stringFormat(currString, 4);
    lcd.print(currString);
    lcd.print("A");



    // LAP TIME PRINTING -------------------------------
    // Calculate elapsed time since last lap
    currentTime = millis();
    elapsedTime = currentTime - lapStartTime;

    // Convert milliseconds to minutes and seconds
    seconds = (elapsedTime / 1000) ;
    minutes = seconds/60;

    lcd.setCursor(0,1);
    // Display minutes and seconds
    lcd.print(minutes);
    lcd.print(":");

    // Print Seconds
    if ((seconds % 60)<10) {
      lcd.print("0");
    }
    lcd.print(seconds % 60);

    // OVERALL TIME PRINTING---------------------------
    // Calculate elapsed time since start of run
    elapsedTime = currentTime - runStartTime;

    // Convert milliseconds to minutes and seconds
    seconds = (elapsedTime / 1000) ;
    minutes = seconds/60;

    lcd.setCursor(6,1);
    // Display minutes and seconds
    lcd.print(minutes);
    lcd.print(":");

    // Print Seconds
    if ((seconds % 60)<10) {
      lcd.print("0");
    }
    lcd.print(seconds % 60);

    lcd.setCursor(13,1);
    lcd.print("L");
    lcd.print(lapNumber);
  }
  else {
    // Need to press lap button to start timers
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press lap button");
    lcd.setCursor(0, 1);
    lcd.print("to start timer");
  }
}

// Pad with zeros to fill allocated LCD characters
String stringFormat(String number, int len){
  while(number.length() < len){
    number = "0" + number;
  }
  return number;
}

// Restart overall timer and lap count
void timerSetup(){
  runStartTime = millis();
  lapStartTime = runStartTime;
  running = true;
  lcd.clear();
}

// ISR For lap button press
// Processing can't be done in function as it 
// broke the LCD writing functions
void lapButtonPress(){
  Serial.println("Button");
  pressed = true;
}

// Processing for button press
void buttonPressed(){
  if (millis() - lastPress > debounceTime){
    lapNumber++;
    Serial.print("LAP: ");
    Serial.println(lapNumber);

    if (millis() - firstPress < resetTimerPressDelay){
      numPresses++;
      if (numPresses > 2){
        // Have had 3 consecutive presses (within resetTimerPressDelay)
        Serial.print("RESET TIMER: ");
        Serial.println(millis() - firstPress);
        restartTimer=true;
      }
    }
    else {
      firstPress = millis(); // Set this press to be first in sequence of 3
      numPresses = 1;
    }

    lastPress = millis();
    lapStartTime = lastPress;

    if (lapNumber == 1){
      timerSetup();
    }
  }  
}