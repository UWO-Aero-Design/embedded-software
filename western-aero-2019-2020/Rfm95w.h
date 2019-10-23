/*
 * 
 */

 #pragma once

#include "Arduino.h"
#include "src/Rfm95w/RH_RF95.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"

using namespace aero;
using namespace aero::teensy35;
using namespace aero::def;

class Rfm95w{
  public:
    Rfm95w();
    ~Rfm95w();
    
    void init();
    void send(char *msg);
    
    String type = String("This is a radio");

  private: 
    RH_RF95 *rf95;
    const unsigned int RESET_PIN = P32, INTERRUPT_PIN = P32;
};
