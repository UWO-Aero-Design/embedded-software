#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION SystemSelect::ZTRDemo1Gnd

System *sys = NULL;

void setup() {
  // create the system specified by user input
  sys = SystemSelect::system_select(SYSTEM_SELECTION);
  sys->init();
  
  if(!Serial) {
    Serial.begin(9600);
  }
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_system_name(SYSTEM_SELECTION));
  Serial.println(" mode");
  
}

void loop() {
  sys->update();
  //delay(1000);
}
