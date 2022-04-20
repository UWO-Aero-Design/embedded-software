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

  private:
    uint32_t animation_count = 0;
    Animation *head = NULL, *tail = NULL;
    
};
