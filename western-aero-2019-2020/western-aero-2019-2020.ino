#include "Arduino.h"
#include "Test_System.h"
#include "Comp_System.h"
#include "System_Select.h"
#include "libraries/MPU9250/MPU9250.h"

System *sys;

#define SYSTEM_SELECTION 0b00001111

void setup() {
  Serial.begin(9600);
}

void loop() {
  
}
