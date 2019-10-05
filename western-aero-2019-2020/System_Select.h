/*
 * implementation of a system that is used during competition
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Test_System.h"
#include "Comp_System.h"

class System_Select {
  public:
    System_Select();
    ~System_Select();
    enum System_Type { Comp_System_t = 0b00001111, Test_System_t = 0b00000000 };
    static System *system_select(uint8_t type) {
      switch(type) {
        case 0b00000000:
          return new Test_System();
          break;
        case 0b00001111:
        default:
          return new Comp_System();
          break;
      }
    };
};
