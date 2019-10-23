#include "Arduino.h"
#include "System.h"

System *sys;

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION SystemSelect::CompSystem_t

void setup() {
  Serial.begin(9600);

  // create the system specified by user input
  sys = SystemSelect::system_select(SYSTEM_SELECTION);
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_system_name(SYSTEM_SELECTION));
  Serial.println(" mode");
  sys->init();
}

void loop() {

  // update the system - temporary delay for debugging purposes
  sys->update();
  delay(1000);
}
