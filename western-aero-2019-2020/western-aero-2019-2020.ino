#include "System_Select.h"
#include "txSerialSystem.h"

#define led 13

txSerial tx_test;

void setup() {
  tx_test.initSystem();

  pinMode(led, OUTPUT);
  Serial.println("Hello World!");
  
}

void loop() {
  tx_test.updateSystem();
}
