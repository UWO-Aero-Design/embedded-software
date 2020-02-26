/** \file western-aero-2019-2020.ino
 * @brief Main file that controls all system creations
 */


#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
const uint8_t DIP_SWITCHES[] = { 24, 25, 26, 27 };

uint8_t system_selection = 0;
// uint8_t system_selection = SystemSelect::GroundStation_t;
System *sys = NULL;

long last_update = 0;
bool state = false;

void setup() {
  Serial.begin(DEFAULT_BAUD);

//  // read dip switches
  for(int i = 3; i >= 0; i--) {
    pinMode(DIP_SWITCHES[i], INPUT);
    system_selection = system_selection | (digitalRead(DIP_SWITCHES[i]) << i);
  }
  
  Serial.print("Booting in ");
  Serial.print(SystemSelect::get_description(system_selection));
  Serial.println(" mode\n");

  // create the system specified by user input
  sys = SystemSelect::select(system_selection);
  if(sys->init()) {
    Serial.println("\nSystem successfully started.\n\n");
  }
  else {
    Serial.println("\nSystem started with errors.\n\n");
  }
  
  for(int i = 20; i < 24; i++) pinMode(i, OUTPUT);
}

void loop() {
  if(state == HIGH && millis() - last_update >= 1) {
    digitalWrite(23, LOW);
    state = LOW;
    last_update = millis();
  }
  if(state == LOW && millis() - last_update >= 500) {
    digitalWrite(23, HIGH);
    state = HIGH;
    last_update = millis();
  }
  sys->update();
}
