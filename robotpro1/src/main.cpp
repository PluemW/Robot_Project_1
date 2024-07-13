#include <Arduino.h>
#include "BluetoothSerial.h"
#include "config.h"
#include "motor.h"

String device_name = bt_device;
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}