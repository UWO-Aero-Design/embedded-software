/*
*  This sketch will be used on the navigation Teensy for flight stabilization
*  This code will receive IMU information from the Main Teensy and measure pwm signals
*  from an RC receiver. It will then output a filtered response to servos located on 
*  control surfaces (Rudder, Elevator, and two Aileron servos). 
*/

#include <Servo.h>

//LED Pins
const uint8_t redLED = 29;    //Used to indicate failed I2C initalization with main Teensy
const uint8_t blueLED = 30;   //Used as a heartbeat
const uint8_t whiteLED = 31;  //Used to indicate flight stabilization is on
const uint8_t yellowLED = 32;

//Input for raw data from RC receiver
const uint8_t RAW_RUDDER = 3;
const uint8_t RAW_ELEVATOR = 5;
const uint8_t RAW_AILERON1 = 7;
const uint8_t RAW_AILERON2 = 9;

//Servo Objects
Servo rudder_S;     //Rudder pin 4
Servo elevator_S;   //Elevator pin 4
Servo aileron1_S;   //Aileron 1 pin 4
Servo aileron2_S;   //Aileron 2 pin 4

//Variables for I2C communication with main Teensy

//Stores output PWM values [rudder, elevator, aileron1, aileron2]
struct pwm_Values 
{
  volatile int rudder = 0;
  volatile int elevator = 0;
  volatile int aileron1 = 0;
  volatile int aileron2 = 0;
} raw_Data, filtered_Data;

volatile int prev_time_rudder = 0;
volatile int prev_time_elevator = 0;
volatile int prev_time_aileron1 = 0;
volatile int prev_time_aileron2 = 0;

//Keep false until set true by the Main Teensy
bool flightStabilization = false;

void setup() {
  pinMode (redLED, OUTPUT);
  pinMode (blueLED, OUTPUT);
  pinMode (whiteLED, OUTPUT);
  pinMode (yellowLED, OUTPUT);

  //Initialize I2C communication with Master Teensy - if not true throw red LED
  
  //Attach Servos
  rudder_S.attach(4);
  elevator_S.attach(6);
  aileron1_S.attach(8);
  aileron2_S.attach(10); 

  //Initialize pwm_Values;
  filtered_Data.rudder = 0;
  filtered_Data.elevator = 0;
  filtered_Data.aileron1 = 0;
  filtered_Data.aileron2 = 0;

  attachInterrupt(digitalPinToInterrupt(RAW_RUDDER), rud_Rising, RISING);
  attachInterrupt(digitalPinToInterrupt(RAW_ELEVATOR), elev_Rising, RISING);
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON1), ail1_Rising, RISING);
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON2), ail2_Rising, RISING);
}

void loop() { 
  //If flight stabilization is true, pass the filtered values through
  if (flightStabilization)
  {
    filterData (raw_Data);
    rudder_S.write(filtered_Data.rudder);
    elevator_S.write(filtered_Data.elevator);
    aileron1_S.write(filtered_Data.aileron1);
    aileron2_S.write(filtered_Data.aileron2);
  }
}

//Pass by reference a struct that contains raw servo positions
void filterData(pwm_Values &raw_Data){

}

//Deciphers instructions from Master Teensy
void decipherMasterMessage(){
  //Set mode of stabilizing flight control
  //Update IMU values
}

//Interrupt subroutines to read the 4 input pins
void rud_Rising()
{
  prev_time_rudder = micros();
  attachInterrupt(digitalPinToInterrupt(RAW_RUDDER), rud_Falling, FALLING);
}
void rud_Falling()
{
  attachInterrupt(digitalPinToInterrupt(RAW_RUDDER), rud_Rising, RISING);
  raw_Data.rudder = micros() - prev_time_rudder;
}
void elev_Rising()
{
  prev_time_elevator = micros();
  attachInterrupt(digitalPinToInterrupt(RAW_ELEVATOR), elev_Falling, FALLING);
}
void elev_Falling()
{
  attachInterrupt(digitalPinToInterrupt(RAW_ELEVATOR), elev_Rising, RISING);
  raw_Data.elevator = micros() - prev_time_elevator;
}
void ail1_Rising()
{
  prev_time_aileron1 = micros();
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON1), ail1_Falling, FALLING);
}
void ail2_Falling()
{
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON1), ail1_Rising, RISING);
  raw_Data.aileron1 = micros() - prev_time_aileron1;
}
void ail2_Rising()
{
  prev_time_aileron2 = micros();
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON2), ail2_Falling, FALLING);
}
void ail2_Falling()
{
  attachInterrupt(digitalPinToInterrupt(RAW_AILERON2), ail2_Rising, RISING);
  raw_Data.aileron2 = micros() - prev_time_aileron2;
}
