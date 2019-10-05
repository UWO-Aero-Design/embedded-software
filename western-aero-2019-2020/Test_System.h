/*
 * implementation of a system that is used for a full system test
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Arduino.h"
#include "System.h"

class Test_System : public System {
  public:
    Test_System();
    ~Test_System();
    void initSystem();
    void updateSystem();
  
  protected:

  
  private:
    String type = String("This is a test system");
  
};
