#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
const uint8_t DIP_SWITCHES[] = { 24, 25, 26, 27 };

// will eventually be read from 4 pos dip switch
#define SYSTEM_SELECTION SystemSelect::CompSystem_t
uint8_t selection = 0;

System *sys = NULL;

void setup() {
  if(!Serial) {
    Serial.begin(DEFAULT_BAUD);
  }
  for(int i = 3; i >= 0; i--) {
    pinMode(DIP_SWITCHES[i], INPUT);
    selection = selection | (digitalRead(DIP_SWITCHES[i]) << i);
  }
  Serial.println(selection, BIN);
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(selection));
  Serial.println(" mode");
  
  // create the system specified by user input
  sys = SystemSelect::select(selection);
  sys->init();
}

void loop() {
  sys->update();
}
