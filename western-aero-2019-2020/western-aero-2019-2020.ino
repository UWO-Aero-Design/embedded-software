/** \file western-aero-2019-2020.ino
 * @brief Main file that controls all system creations
 */

#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
//First 3 switches used to select system type while last switch used to select flight stabilization
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
bool flight_stable = false;

void setup() {

  #ifndef GROUND_STATION
  // read dip switches (26, 25, 24) to boot into appropriate system
  for(int i = 2; i >= 0; i--) {
    pinMode(DIP_SWITCHES[i], INPUT);
    system_selection = system_selection | (digitalRead(DIP_SWITCHES[i]) << i);
  }
  // read flight stabilization mode from dip switch (27)
  pinMode(DIP_SWITCHES[3], INPUT);
  flight_stable = digitalRead(DIP_SWITCHES[3]);
  
  #endif
  sys = SystemSelect::select(system_selection);
  
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(system_selection));
  Serial.print(" mode with flight stabilization ");
  Serial.println(flight_stable);

  for(int i = 20; i < 24; i++) pinMode(i, OUTPUT);
  digitalWrite(20, HIGH);

  // create the system specified by user input
  if(sys->init()) {
    Serial.println("\nSystem successfully started.\n\n");
  }
  else {
    Serial.println("\nSystem started with errors.\n\n");
  }
  
}

void loop() {
  if(state && millis() - last_update >= 100) {
    state = !state;
    digitalWrite(23, state);
    last_update = millis();
  }
  if(!state && millis() - last_update >= 500) {
    state = !state;
    digitalWrite(23, state);
    last_update = millis();
  }
  sys->update();
}
