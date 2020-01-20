#pragma once

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/ServoDriver/Adafruit_PWMServoDriver.h"
#include "ServoDef.hpp"

using namespace aero::def;
using namespace aero::servos;

/**
 * @brief Controls the servos for the onboard system
 */
class ServoController{
  public:
  
    ServoController() { };
    ~ServoController() { };

    /**
     * @brief Initialize the servo controller
     */
    void init() {
      m_pwm.begin();
      m_pwm.setOscillatorFrequency(27000000);
      m_pwm.setPWMFreq(1600);
    };

    /**
     * @brief Open a servo
     * 
     * @param servo The servo of interest
     */
    void actuate_servo(Servo_t servo, int position) {
      m_pwm.setPWM(servo.pin, 0, position);
    };

    /**
     * @brief Open all servos
     */
    void open_all() {
      for(int i = 0; i < 16; i++) m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].open_pos);
    }

    /**
     * @brief Close all servos
     */
    void close_all() {
      for(int i = 0; i < 16; i++) m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].open_pos);
    }

    /**
     * @brief Perform a drop command
     * 
     * @param id The id of the payload to drop
     */
    void drop_payload(PayloadId id) {
      for(int i = 0; i < sizeof(m_servos)/sizeof(Servo_t); i++) {
        if(m_servos[i].id == id && m_servos[i].id != NONE) m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].open_pos);
      }
    }

  private: 
    Adafruit_PWMServoDriver m_pwm;
    Servo_t m_servos[16] = { servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11, servo12, servo13, servo14, servo15 };
    
};
