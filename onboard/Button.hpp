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
        buttons[i].pin = pins[i];
        buttons[i].last_state = digitalRead(buttons[i].pin);
        buttons[i].last_update = 0;
        buttons[i].on_positive = [this](int button_number, void *context) {};
        buttons[i].on_negative = [this](int button_number, void *context) {};
        buttons[i].on_either = [this](int button_number, void *context) {};
        pinMode(buttons[i].pin, INPUT);
      }
    };

    bool update() {
      for(int i = 0; i < BUTTON_COUNT; i++) {
        bool state = digitalRead(buttons[i].pin);
        if(buttons[i].last_state != state && millis() - buttons[i].last_update >= BUTTON_DEBOUNCE) {
          if(state == HIGH) {
            buttons[i].on_positive(buttons[i].pin, this);
          }
          else {
            buttons[i].on_negative(buttons[i].pin, this);
          }
          buttons[i].on_either(i, this);
          buttons[i].last_state = !buttons[i].last_state;
          buttons[i].last_update = millis();
        }
      }
      return true;
    }

    bool on(uint8_t pin_number, TransitionType_t transition, std::function<void(uint8_t pin_number, void *context)> action) {
      Button_t *btn;
      btn = get_button_by_pin(pin_number);
      if(btn == NULL) {
        Serial.println("Error: invalid button pin");
        return false;
      }
      
      if(transition == TransitionType_t::RISING_EDGE) {
        btn->on_positive = action;
      }
      else if(transition == TransitionType_t::FALLING_EDGE) {
        btn->on_negative = action;
      }
      else if(transition == TransitionType_t::EITHER_EDGE) {
        btn->on_either = action;
      }
      return true;
    }

    Button_t *get_button_by_pin(uint8_t pin) {
      for(int i = 0; i < BUTTON_COUNT; i++) {
        if(buttons[i].pin == pin) return &buttons[i];
      }

      return NULL;
    }

  private:
    Button_t buttons[BUTTON_COUNT];
    uint8_t pins[BUTTON_COUNT] = { Pins::BUTTON_1, Pins::BUTTON_2 };
    
};
