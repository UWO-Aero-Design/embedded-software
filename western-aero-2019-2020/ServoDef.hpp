/** \file ServoDef.hpp
 * @brief Calibration of the servos
 */

#pragma once
    
enum PayloadId { DOOR, PAYLOAD0, PAYLOAD1, PAYLOAD2, GLIDER0, GLIDER1, NONE };

/*!
  @brief Representation of a servo
*/
struct Servo_t {
  uint8_t pin;
  uint16_t open_pos;
  uint16_t closed_pos;
  PayloadId id;
};

Servo_t servo0  = { 0, 0, 0, NONE };
Servo_t servo1  = { 0, 0, 0, NONE };
Servo_t servo2  = { 0, 0, 0, NONE };
Servo_t servo3  = { 0, 0, 0, NONE };
Servo_t servo4  = { 0, 0, 0, NONE };
Servo_t servo5  = { 0, 0, 0, NONE };
Servo_t servo6  = { 0, 0, 0, NONE };
Servo_t servo7  = { 0, 0, 0, NONE };
Servo_t servo8  = { 0, 0, 0, NONE };
Servo_t servo9  = { 0, 0, 0, NONE };
Servo_t servo10 = { 0, 0, 0, NONE };
Servo_t servo11 = { 0, 0, 0, NONE };
Servo_t servo12 = { 0, 0, 0, NONE };
Servo_t servo13 = { 0, 0, 0, NONE };
Servo_t servo14 = { 0, 0, 0, NONE };
Servo_t servo15 = { 0, 0, 0, NONE };
