/** \file Servo.hpp
 * @brief All code relating to the servo controller
 */

#pragma once

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/ServoDriver/Adafruit_PWMServoDriver.h"
#include "ServoDef.hpp"
#include "Wire.h"

/**
 * @brief 16-channel servo controller
 * @details Communication to the controller is handled via I2C
 */
class ServoController{
  public:
  
    ServoController() { };
    ~ServoController() { };

    // array of all 16 servos (0-15)
    Servo_t m_servos[16] = { servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11, servo12, servo13, servo14, servo15 };

    /**
     * @brief Initialize the servo controller
     */
    bool init() {

      // quickly begin and end a transmission to determine if chip is connected
      Wire.beginTransmission(ADDRESS);
      int result = Wire.endTransmission();

      // wire returns '0' if the transmission was ack'd
      if(result != 0) {
        m_initilized = false;
      }
      else {
        m_initilized = true;
      }

      // init pwm library
      m_pwm.begin();
      m_pwm.setOscillatorFrequency(27000000);
      m_pwm.setPWMFreq(50); // frequebcy of most analog servos
      
      return m_initilized;
    };

    /**
     * @brief Open a servo
     * 
     * @param servo The servo of interest
     */
    void actuate_servo(Servo_t servo, int position) {
      m_pwm.setPWM(servo.pin, 0, position);
    };

    void open_servo(Servo_t servo) {
      m_pwm.setPWM(servo.pin, 0, servo.open_pos);
    };

    void close_servo(Servo_t servo) {
      m_pwm.setPWM(servo.pin, 0, servo.open_pos);
    };

    /**
     * @brief Open all assigned servos
     */
    void open_all() {
      for(int i = 0; i < 16; i++) {
        // only open servos with an assignment
        if(m_servos[i].id != NONE) {
          m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].open_pos);
        }
      }
    }

    /**
     * @brief Close all assigned servos
     */
    void close_all() {
      for(int i = 0; i < 16; i++) {
        if(m_servos[i].id != NONE) {
          // only close servos with an assignment
          m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].close_pos);
        }
      }
    }

    /**
     * @brief Perform a command
     * 
     * @param id The id of the command to perform
     */
    void actuate(CommandId id) {
      for(int i = 0; i < sizeof(m_servos)/sizeof(Servo_t); i++) {
        // open the servo only if it is assigned to the specified id
        if(m_servos[i].id == id && m_servos[i].id != NONE) {
          m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].open_pos);
        }
      }
    }

    /**
     * @brief Undo a command
     * 
     * @param id The id of the command to undo
     */
    void reset(CommandId id) {
      for(int i = 0; i < sizeof(m_servos)/sizeof(Servo_t); i++) {
        // close the servo only if it is assigned to the specified id
        if(m_servos[i].id == id && m_servos[i].id != NONE) m_pwm.setPWM(m_servos[i].pin, 0, m_servos[i].close_pos);
      }
    }

  private:
    constexpr static int ADDRESS = 0x47; // defined based off of v2.0 of onboard systems PCB
    bool m_initilized = false; // keep track of init state
    Adafruit_PWMServoDriver m_pwm = Adafruit_PWMServoDriver(ADDRESS); // servo driver object
    
};
