
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String name = "Magic Spoon";
bool connected;

#define RXD2 16
#define TXD2 17

void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("ESP32 Master", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  
  connected = SerialBT.connect(name);
  digitalWrite(2, LOW);
  if(connected) {
    digitalWrite(2, HIGH);
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
}

void loop() {
  if (SerialBT.available()) {
    byte ctr = SerialBT.read();
    Serial.write(ctr);
    Serial2.write(ctr);
  }
//  delay(20);
}
