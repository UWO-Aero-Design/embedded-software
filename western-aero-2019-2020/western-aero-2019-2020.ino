#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
const uint8_t DIP_SWITCHES[] = { 24, 25, 26, 27 };

uint8_t system_selection = 0;

System *sys = NULL;

void setup() {
  Serial.begin(DEFAULT_BAUD);
  while(!Serial) { }

  // read dip switches
  for(int i = 3; i >= 0; i--) {
    pinMode(DIP_SWITCHES[i], INPUT);
    system_selection = system_selection | (digitalRead(DIP_SWITCHES[i]) << i);
  }
  
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(system_selection));
  Serial.println(" mode");
  
  // create the system specified by user input
  sys = SystemSelect::select(system_selection);
  sys->init();
  if(true) {
    Serial.println("System successfully started.\n\n");
  }
  else {
    Serial.println("System started with errors.\n\n");
  }
}

void loop() {
  sys->update();
}
