#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("MyESP32"); // Super simple name
  
  Serial.println("Bluetooth Test Ready!");
  Serial.println("Look for: MyESP32");
  Serial.println("Send any message via Bluetooth");
}

void loop() {
  // Echo between Bluetooth and Serial
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.write(c);
    SerialBT.write(c); // Echo back
  }
  
  if (Serial.available()) {
    char c = Serial.read();
    SerialBT.write(c);
  }
  
  delay(10);
}