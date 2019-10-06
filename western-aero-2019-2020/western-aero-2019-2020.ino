#include "Arduino.h"
#include "System.h"

System *sys;

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION SystemSelect::TestSystem_t

void setup() {
  Serial.begin(9600);

  // create the system specified by user input
  sys = SystemSelect::system_select(SYSTEM_SELECTION);
  
  sys->init();
}

void loop() {

  // update the system - temporary delay for debugging purposes
  sys->update();
  delay(1000);
}
