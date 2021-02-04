/** \file western-aero-2019-2020.ino
 * @brief Main file that controls all system creations
 */

#include "Arduino.h"
#include "System.hpp"

const uint8_t BUILTIN_LED = 13;
const int DEFAULT_BAUD = 9600;
//First 3 switches used to select system type while last switch used to select flight stabilization
const uint8_t DIP_SWITCHES[] = { 24, 25, 26, 27 };
const uint8_t AUX_CHANNEL = 12;

//aux_value stores the PWM input from the aux channel
volatile int aux_value = 0;
//prev_time stores the last time that a rising edge was noticed on the aux-channel
volatile long prev_time = 0;

System *sys = NULL;

#ifdef GROUND_STATION
uint8_t system_selection = SystemSelect::GroundStation_t;
#endif
#ifndef GROUND_STATION
uint8_t system_selection = 0;
#endif

//Variables are used for the LED light sequence
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
  
  //Read flight stabilization mode from dip switch (pin 27)
  pinMode(DIP_SWITCHES[3], INPUT);
  flight_stable = digitalRead(DIP_SWITCHES[3]);
  if (flight_stable){
    attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), rising, RISING);
    //Initialize i2c comms with nav teensy here

  }
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
  //This sequence of code is used to generate a cool light sequence for the plane 
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
  //Update the system
  sys->update();
}

//ISR for rising edge saves the time in microseconds and attachs interrupt to falling edge
void rising() {
  prev_time = micros();
  attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), falling, FALLING)
}

//ISR for falling edge finds the pulse width of the PWM signal and resets interrupt to rising edge
void falling() {
  aux_value = micros() - prev_time;
  attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), rising, RISING)
}