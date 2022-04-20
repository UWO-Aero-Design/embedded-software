/** \file Led.hpp
   @brief All code relating to controlling the LEDs
*/


#pragma once

class Animation {
  public:
    Animation *next = NULL;
    
    Animation(){};
    virtual ~Animation(){};
    virtual bool update() = 0;

    bool is_enabled() {
      return enabled;
    }

    void set_enabled(bool enabled) {
      this->enabled = enabled;
    }

  protected:
    uint32_t pin;
    uint32_t enabled = true;
};
