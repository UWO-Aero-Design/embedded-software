#include "Arduino.h"
#include "System_Select.h"

System *sys;

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION System_Select::Test_System_t

void setup() {
  Serial.begin(9600);

  // create the system specified by user input
  sys = System_Select::system_select(SYSTEM_SELECTION);
  
  sys->initSystem();
}

void loop() {

  // update the system - temporary delay for debugging purposes
  sys->updateSystem();
  delay(1000);
}
