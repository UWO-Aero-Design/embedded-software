/** \file western-aero-2019-2020.ino
 * @brief Main file that controls all system creations
 */


#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
const uint8_t DIP_SWITCHES[] = { 24, 25, 26, 27 };

System *sys = NULL;

#ifdef GROUND_STATION
uint8_t system_selection = SystemSelect::GroundStation_t;
#endif
#ifndef GROUND_STATION
uint8_t system_selection = 0;
#endif

long last_update = 0;
bool state = false;

void setup() {

  #ifndef GROUND_STATION
  // read dip switches
  for(int i = 3; i >= 0; i--) {
    pinMode(DIP_SWITCHES[i], INPUT);
    system_selection = system_selection | (digitalRead(DIP_SWITCHES[i]) << i);
  }
  #endif
  sys = SystemSelect::select(system_selection);
  
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(system_selection));
  Serial.println(" mode\n");

  // create the system specified by user input
  if(sys->init()) {
    Serial.println("\nSystem successfully started.\n\n");
  }
  else {
    Serial.println("\nSystem started with errors.\n\n");
  }
  
  for(int i = 20; i < 24; i++) pinMode(i, OUTPUT);
}

void loop() {
  sys->update();
}
