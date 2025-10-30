
#include <Wire.h>

#define TXD2 2  // RX (on Wio-E5)
#define RXD2 3  // TX (on Wio-E5)

String receivedHex = "";
String receivedString = "";

void setup() {
  Serial.begin(9600);  // For debugging
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);   // Communication with Wio-E5

  Serial.println("Sending AT commands to Wio-E5...");

  Serial1.print("AT+MODE=TEST\r\n");  // Set to test mode
  //delay(1000);
  //printResponse();

  Serial1.print("AT+TEST=RFCFG,868,7,125,12,15,22,ON,OFF,OFF\r\n");  // Configure RF
  //delay(1000);
  //printResponse();

  Serial1.print("AT+TEST=RXLRPKT\r\n");  // Receive data
  //delay(1000);
  //printResponse();
}

void loop() {
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');  // Read until newline
    //Serial.println("Raw Response: " + response);

    if (response.startsWith("+TEST: RX")) {
      int hexStart = response.indexOf("RX") + 3;
      receivedHex = response.substring(hexStart);
      receivedHex.trim();

      if (receivedHex.startsWith("\"") && receivedHex.endsWith("\"")) {
        receivedHex = receivedHex.substring(1, receivedHex.length() - 1);
      }

      receivedString = hexToAscii(receivedHex);
      Serial.println(receivedString);
      Serial.flush();
      delay(100);
      
    }
  }
}

  // Convert HEX string to ASCII
  String hexToAscii(String hex) {
    String ascii = "";
    for (int i = 0; i < hex.length(); i += 2) {
      String byteStr = hex.substring(i, i + 2);
      char c = strtol(byteStr.c_str(), NULL, 16);
      ascii += c;
    }
    return ascii;
  }

void printResponse() {
  // Flush any existing data
  while (Serial1.available()) {
    Serial1.read();
  }
  delay(100);  // Give some time for the complete response to arrive
  String response = readCompleteResponse();  
  if (response.length() > 0) {
    Serial.print("Wio-E5 Response: ");
    Serial.println(response);
  } else {
    Serial.println("No complete response received.");
  }
}

String readCompleteResponse() {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 1500) {  // Wait up to 1.5 seconds
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
      if (c == '\n') {
        return response;
      }
    }
  }
  return response;
}

