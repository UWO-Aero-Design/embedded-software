/** \file Button.hpp
   @brief All code relating to reading from the buttons
*/


#pragma once

#include <functional>
#include "Pins.hpp"

#define BUTTON_COUNT 2
#define BUTTON_DEBOUNCE 100

typedef struct {
  uint8_t pin;
  bool last_state;
  uint32_t last_update;
  std::function<void(int button_number, void *context)> on_positive;
  std::function<void(int button_number, void *context)> on_negative;
  std::function<void(int button_number, void *context)> on_either;
} Button_t;

typedef enum {
  RISING_EDGE,
  FALLING_EDGE,
  EITHER_EDGE
} TransitionType_t;


/**
   @brief Class for attaching callbacks to button clicks
   @details 
*/
class ButtonController {
  public:

    /**
       @brief Construct a new Led controller object
    */
    ButtonController() {
      for(int i = 0; i < BUTTON_COUNT; i++) {
        pinMode(buttons[i].pin, INPUT);
        buttons[i].pin = pins[i];
        buttons[i].last_state = digitalRead(buttons[i].pin);
        buttons[i].last_update = 0;
        buttons[i].on_positive = [this](int button_number, void *context) {};
        buttons[i].on_negative = [this](int button_number, void *context) {};
        buttons[i].on_either = [this](int button_number, void *context) {};
      }
    };

    bool update() {
      for(int i = 0; i < BUTTON_COUNT; i++) {
//        bool state = digitalRead(buttons[i].pin);
//        if(buttons[i].last_state != state && millis() - buttons[i].last_update >= BUTTON_DEBOUNCE) {
//          if(state == HIGH) {
//            Serial.print("Got HIGH state change for button_");
//            Serial.println(i);
////            buttons[i].on_positive(i, this);
//          }
//          else {
//            Serial.print("Got LOW state change for button_");
//            Serial.println(i);
////            buttons[i].on_negative(i, this);
//          }
////          buttons[i].on_either(i, this);
//          buttons[i].last_state = !buttons[i].last_state;
//          buttons[i].last_update = millis();
//        }
      }
      return true;
    }

    bool on(TransitionType_t transition, uint8_t button_number, std::function<void(int button_number, void *context)> action) {
//      if(transition == TransitionType_t::RISING_EDGE) {
//        Serial.println("Setting rising edge");
//        buttons[button_number].on_positive = action;
//      }
//      else if(transition == TransitionType_t::FALLING_EDGE) {
//        Serial.println("Setting falling edge");
//        buttons[button_number].on_negative = action;
//      }
//      else if(transition == TransitionType_t::EITHER) {
//        Serial.println("Setting either edge");
//        buttons[button_number].on_either = action;
//      }
      return true;
    }

  private:
    Button_t buttons[BUTTON_COUNT];
    uint8_t pins[BUTTON_COUNT] = { Pins::BUTTON_1, Pins::BUTTON_2 };
    
};
