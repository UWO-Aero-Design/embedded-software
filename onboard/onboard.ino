/** \file western-aero-2019-2020.ino
 * @brief Main file that controls all system creations
 */

#include "Arduino.h"
#include "System.hpp"
#include "Wire.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Pins.hpp"
#include "Addresses.hpp"

// comment to use dip switches for system select
// uncomment to use comp system
#define OVERRIDE_SYS_TO_COMP_SYS true

const long BAUD_RATE = 115200;

// first 3 switches used to select system type while last switch used to select flight stabilization
const uint8_t DIP_SWITCHES[] = { Pins::DIPSWITCH_1, Pins::DIPSWITCH_2, Pins::DIPSWITCH_3, Pins::DIPSWITCH_4 };
const uint8_t AUX_CHANNEL = Pins::AUX;

bool nav_initialized = false;      //Stores state if nav board is initalized or not

//aux_value stores the PWM input from the aux channel
volatile int aux_value = 0;
//prev_time stores the last time that a rising edge was noticed on the aux-channel
volatile long prev_time = 0;

System *sys = NULL;

uint8_t system_selection = 0;

//Variables are used for the LED light sequence
long last_update = 0;
bool state = false;
bool flight_stable = false;

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);

  pinMode(DIPSWITCH_1, INPUT);
  pinMode(DIPSWITCH_2, INPUT);
  pinMode(DIPSWITCH_3, INPUT);
  pinMode(DIPSWITCH_4, INPUT);

  #ifdef OVERRIDE_SYS_TO_COMP_SYS
    // use comp system
    system_selection = SystemSelect::SystemType::CompSystem_t;
  #elif
  // read dip switches (1-3) to boot into appropriate system
    system_selection = system_selection | digitalRead(Pins::DIPSWITCH_1);
    system_selection = system_selection | digitalRead(Pins::DIPSWITCH_2) << 1);
    system_selection = system_selection | digitalRead(Pins::DIPSWITCH_3) << 2);
  #endif

  // select system
  sys = SystemSelect::select(system_selection);

  // read flight stabilization mode from dip switch 4
//  pinMode(DIP_SWITCHES[3], INPUT);
//  flight_stable = digitalRead(Pins::DIPSWITCH_4);
  
  Serial.print("Booting in ");
  Serial.println(SystemSelect::get_description(system_selection));
//  Serial.print(" mode with flight stabilization ");
//  Serial.println(flight_stable);

  // init the created system
  if(sys->init()) {
    Serial.println("\nSystem successfully started.\n\n");
  }
  else {
    Serial.println("\nSystem started with errors.\n\n");
  }
//  
//  //Adjustments to system selection based on if flight_stabilization is enabled
//  if (flight_stable){
//    attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), rising, RISING);
//    
//    //Initialize i2c with Navigation Teensy
//    Wire.beginTransmission(Addresses::NAV_BOARD);
//    int response = Wire.endTransmission();
//
//    if(response != 0){
//      nav_initialized = false;
//      Serial.println ("Comms not established with nav board");
//    }
//    else{
//      nav_initialized = true;
//      Serial.println ("Comms established with nav board");
//    }
//  }
}

void loop() {
  //Update the system
  sys->update();
//  //This sequence of code is used to generate a cool light sequence for the plane 
//  if(state && millis() - last_update >= 100) {
//    state = !state;
//    digitalWrite(23, state);
//    last_update = millis();
//  }
//  if(!state && millis() - last_update >= 500) {
//    state = !state;
//    digitalWrite(23, state);
//    last_update = millis();
//  }
//
//  //Update nav_board with status of flight control
//  if (flight_stable){
//    Wire.beginTransmission(Addresses::NAV_BOARD);
//    //If aux_value > 1750, switch is ON. If less than 1750, switch is OFF
//    if (aux_value>1750){
//      Wire.write(1);
//    }
//    else{
//      Wire.write(0);
//    }
//    Wire.endTransmission();
//  }
}

//ISR for rising edge saves the time in microseconds and attachs interrupt to falling edge
//void rising() {
//  prev_time = micros();
//  attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), falling, FALLING);
//}

//ISR for falling edge finds the pulse width of the PWM signal and resets interrupt to rising edge
//void falling() {
//  aux_value = micros() - prev_time;
//  attachInterrupt(digitalPinToInterrupt(AUX_CHANNEL), rising, RISING);
//}
