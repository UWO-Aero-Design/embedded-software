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

/**
   @brief Class for controlling LEDs
   @details allows for animating LEDs in an asynchronous manner
*/
class LedController {
  public:

    /**
       @brief Construct a new Led controller object
    */
    LedController() {};

    bool update() {
      Animation *index = head;
      uint32_t count = 0;
      bool update_occured = false;

      while(index != NULL) {
        update_occured |= index->update();
        index = index->next;
      }

      return update_occured;
    }

    bool attach(Animation *animation) {
      if(head == NULL) {
        head = animation;
      }
      else {
        Animation *index = head;
        if(index == animation) {
          Serial.println("Error: Animation already exists");
          return false;
        }
        
        while(index->next != NULL) {
          if(index == animation) {
            Serial.println("Error: Animation already exists");
            return false;
          }
          index = index->next;
        }
        index->next = animation;
      }
      animation_count++;
      return true;
    }

//    bool detach(Animation *animation) {
//      if(head == null) {
//        return false;
//      }
//      else {
//        Animation *index = head;
//        Animation *previous = NULL;
//
//        if(index != NULL) {
//          if(index == animation) {
//            head = index->next;
//            animation_count--;
//            return true;
//          }
//          else {
//            while(index != NULL && index != animation) {
//              previous = index;
//              index = index->next;
//            }
//            if(index != NULL) {
//              previous->next = index->next;
//              animation_count--;
//              return true;
//            }
//          }
//        }
//        return false;
//      }
//    }

  private:
    uint32_t animation_count = 0;
    Animation *head = NULL, *tail = NULL;
    
};

class BlinkAnimation : public Animation {
  public:
  
    BlinkAnimation(uint32_t pin, uint32_t on_time, uint32_t off_time, bool inital_state = LOW) : on_time(on_time), off_time(off_time), state(inital_state) {
      this->pin = pin;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, state);
    };
    ~BlinkAnimation(){};
  
    bool update() {
      if(!enabled) {
        return false;
      }
      
      if(state == HIGH && millis() - last_blink >= on_time) {
          state = LOW;
          digitalWrite(pin, state);
          last_blink = millis();
          return true;
      }
      else if(state == LOW && millis() - last_blink >= off_time) {
        state = HIGH;
        digitalWrite(pin, state);
        last_blink = millis();
        return true;
      }
      
      return false;
    }

    void set_on_time(uint32_t on_time) {
      this->on_time = on_time;
    }

    void set_off_time(uint32_t off_time) {
      this->off_time = off_time;
    }

  private:

    uint32_t on_time;
    uint32_t off_time;
    uint32_t last_blink = 0;
    bool state;
};

class DoublePulseAnimation : public Animation {
  public:
  
    DoublePulseAnimation(uint32_t pin, uint32_t on_time, uint32_t off_time, uint32_t delay_time, bool inital_state = LOW) : on_time(on_time), off_time(off_time), delay_time(delay_time) {
      this->pin = pin;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, state);
    };
    ~DoublePulseAnimation(){};
  
    bool update() {
      if(!enabled) {
        return false;
      }
      
      if(state == FIRST_ON && millis() - last_update >= on_time) {
        state = SHORT_OFF;
        digitalWrite(pin, LOW);
        last_update = millis();
        return true;
      }
      else if(state == SHORT_OFF && millis() - last_update >= off_time) {
        state = SECOND_ON;
        digitalWrite(pin, HIGH);
        last_update = millis();
        return true;
      }
      else if(state == SECOND_ON && millis() - last_update >= on_time) {
        state = LONG_OFF;
        digitalWrite(pin, LOW);
        last_update = millis();
        return true;
      }
      else if(state == LONG_OFF && millis() - last_update >= delay_time) {
        state = FIRST_ON;
        digitalWrite(pin, HIGH);
        last_update = millis();
        return true;
      }
      
      return false;
    }

    void set_on_time(uint32_t on_time) {
      this->on_time = on_time;
    }

    void set_off_time(uint32_t off_time) {
      this->off_time = off_time;
    }

  private:

    uint32_t on_time;
    uint32_t off_time;
    uint32_t delay_time;
    uint32_t last_update = 0;
    typedef enum { FIRST_ON, SHORT_OFF, SECOND_ON, LONG_OFF } State_t;
    State_t state;
};

class BrightnessAnimation : public Animation {
  public:
  
    BrightnessAnimation(uint32_t pin, uint32_t brightness) {
      this->pin = pin;
      set_brightness(brightness);
      pinMode(pin, OUTPUT);
      analogWrite(pin, brightness);
    };
    ~BrightnessAnimation(){};
  
    bool update() {
      if(!enabled) {
        return false;
      }
      
      if(need_to_update) {
        analogWrite(pin, brightness);
        need_to_update = false;
        return true;
      }

      return false;
    }

    void set_brightness(uint32_t brightness) {
      this->brightness = map(brightness, 0, 100, 0, 255);
      need_to_update = true;
    }

  private:

    uint32_t brightness;
    bool need_to_update = true;
};
