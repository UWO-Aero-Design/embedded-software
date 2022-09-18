/** \file ServoDef.hpp
 * @brief Calibration of the servos
 */

#pragma once

enum CommandId { NONE, PADA };

/*!
  @brief Representation of a servo
*/
struct Servo_t {
  uint8_t pin;
  uint16_t open_pos; // out of 4096
  uint16_t close_pos; // out of 4096
  CommandId id;
};


// defines for servo open, close and command association
//                      OPEN CLOSE  COMMAND
Servo_t servo0  = { 0,  190,  280,  PADA     }; // FRONT RIGHT
Servo_t servo1  = { 1,  350,  250,  PADA     }; // BACK
Servo_t servo2  = { 2,  180,  100,  NONE     };
Servo_t servo3  = { 3,  200,  130,  PADA     }; // FRONT LEFT
Servo_t servo4  = { 4,  100,  130,  NONE     };
Servo_t servo5  = { 5,  100,  130,  NONE     };
Servo_t servo6  = { 6,  100,  130,  NONE     };
Servo_t servo7  = { 7,  100,  130,  NONE     };
Servo_t servo8  = { 8,  100,  130,  NONE     };
Servo_t servo9  = { 9,  100,  130,  NONE     };
Servo_t servo10 = { 10, 100,  130,  NONE     };
Servo_t servo11 = { 11, 100,  130,  NONE     };
Servo_t servo12 = { 12, 100,  130,  NONE     };
Servo_t servo13 = { 13, 100,  130,  NONE     };
Servo_t servo14 = { 14, 100,  130,  NONE     };
Servo_t servo15 = { 15, 100,  130,  NONE     };
