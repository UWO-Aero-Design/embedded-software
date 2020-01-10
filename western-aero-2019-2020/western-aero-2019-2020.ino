#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION SystemSelect::AdafruitGPSDemo_t

System *sys = NULL;

void setup() {
  // create the system specified by user input
  sys = SystemSelect::select(SYSTEM_SELECTION);
  sys->init();
  
  if(!Serial) {
    Serial.begin(DEFAULT_BAUD);
  }

  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(SYSTEM_SELECTION));
  Serial.println(" mode");
}

void loop() {
  sys->update();
}
