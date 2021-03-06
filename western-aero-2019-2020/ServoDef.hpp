/** \file ServoDef.hpp
 * @brief Calibration of the servos
 */

#pragma once

 //                        NONE means it will never be actuated
 //                            HABITAT    WATER    HABITAT
enum CommandId { DOOR0, DOOR1, PAYLOAD0, PAYLOAD1, PAYLOAD2, GLIDER0, GLIDER1, NONE };

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
Servo_t servo0  = { 0,  95 ,  480,  GLIDER0  }; // LEFT  - BLUE
Servo_t servo1  = { 1,  170,  490,  GLIDER1  }; // RIGHT - RED
Servo_t servo2  = { 2,  120,  400,  DOOR0    };
Servo_t servo3  = { 3,  470,  190,  DOOR1    };
Servo_t servo4  = { 4,  230,  420,  PAYLOAD0 };
Servo_t servo5  = { 5,  460,  250,  PAYLOAD1 };
Servo_t servo6  = { 6,  430,  220,  PAYLOAD2 };
Servo_t servo7  = { 7,  180,  400,  NONE     };
Servo_t servo8  = { 8,  180,  400,  NONE     };
Servo_t servo9  = { 9,  180,  400,  NONE     };
Servo_t servo10 = { 10, 180,  400,  NONE     };
Servo_t servo11 = { 11, 180,  400,  NONE     };
Servo_t servo12 = { 12, 180,  400,  NONE     };
Servo_t servo13 = { 13, 180,  400,  NONE     };
Servo_t servo14 = { 14, 180,  400,  NONE     };
Servo_t servo15 = { 15, 180,  400,  NONE     };
