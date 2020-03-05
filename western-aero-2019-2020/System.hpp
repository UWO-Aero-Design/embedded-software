/** \file System.hpp
 * @brief Contains all system base class related code
 */

#pragma once

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"
#include "src/aero-cpp-lib/include/Serial.hpp"
#include "Imu.hpp"
#include "Radio.hpp"
#include "MockData.hpp"
#include "PitotTube.hpp"
#include "Enviro.hpp"
#include "GPS.hpp"
#include "src/Rfm95w/RH_RF95.h"
#include "Servo.hpp"

// #define GROUND_STATION

using namespace aero;
using namespace aero::def;

/**
 * @brief Astract class for defining how a system should be structured
 */
class System {
  public:
    System(){};
    virtual ~System(){};
    virtual bool init() = 0;
    virtual bool update() = 0;
};

#include "CompSystem.hpp"
#include "TestSystem.hpp"
#include "GroundStation.hpp"

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/********************* SYSTEM SELECT MECHANISM/FACTORY *********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Class for performing a factory-styled pattern of system object creation
*/
class SystemSelect {
  public:
    /**
     *  @brief The different valid systems the onboard code can boot into
     */
    enum Type {
      TestSystem_t          = 0b00000000, // System for testing system select
      TesttxSerial_t        = 0b00000001, // System for sending messages based on protocol over serial
      ServoDriverDemo_t     = 0b00000010, // System for testing servo driver
      SerialToRadioDemo_t   = 0b00000011, // System for testing serial to radio routing
      GroundStation_t       = 0b10000000, // System for ground station testing
      PitotDemo_t           = 0b00000100, // System for testing radio for capstone board
      EnviroDemo_t          = 0b00001000, // System for testing the environment sensor
      AdafruitGPSDemo_t     = 0b00001001, // System for testing the adafruit gps module
      RadioClientDemo_t     = 0b00001010, // System for testing the radio in client mode
      RadioServerDemo_t     = 0b00001011, // System for testing the radio in server mode
      IMUDemo_t             = 0b00001100, // System for testing the imu sensor
      RadioWithGliderDemo_t = 0b00001101, // System for testing radio for capstone board
      ZTRDemo1Gnd_t         = 0b11111111, // System for first ZTR target demo
      CompSystem_t          = 0b00001111  // System for competition
    };

     /**
     * @brief Used to get a system object from a valid system type
     *
     * @param type Type of system to return (SystemType)
     * @return System pointer of the correct type
    */
    static System *select(Type type) {
      switch(type) {
        case TestSystem_t: {
          return new TestSystem();
        } break;

        case TesttxSerial_t: {
          return new txSerial();
        } break;

        case CompSystem_t: {
          return new CompSystem();
        } break;

        case ZTRDemo1Gnd_t: {
          return new ZTRDemo1GndStation();
        } break;

        case PitotDemo_t: {
          return new PitotTubeDemo();
        } break;

        case EnviroDemo_t: {
          return new EnviroSensorDemo();
        } break;

        case IMUDemo_t: {
          return new IMUSensorDemo();
        } break;

        case AdafruitGPSDemo_t: {
          return new AdafruitGPSDemo();
        } break;

        case RadioClientDemo_t: {
          return new RadioClientDemo();
        } break;

        case RadioServerDemo_t: {
          return new RadioServerDemo();
        } break;

        case RadioWithGliderDemo_t: {
          return new RadioWithGliderDemo();
        } break;

        case SerialToRadioDemo_t: {
          return new SerialToRadioDemo();
        } break;

        case GroundStation_t: {
          return new GroundStation();
        } break;

        case ServoDriverDemo_t: {
          return new ServoDriverDemo();
        } break;

        default: {
          return new CompSystem();
        } break;
      }
    }

    /**
     * @brief Used to get the name of a system from a valid system type. Purely for printing to the serial monitor.
     *
     * @param type Type of system to return (SystemType)
     * @return String containing the name of the system
     */
    static String get_description(Type type) {
      switch(type) {
        case TestSystem_t: {
          return "Test System";
        } break;

        case TesttxSerial_t: {
          return txSerial::DESCRIPTION;
        } break;

        case ZTRDemo1Gnd_t: {
          return ZTRDemo1GndStation::DESCRIPTION;
        } break;

        case PitotDemo_t: {
          return PitotTubeDemo::DESCRIPTION;
        } break;

        case EnviroDemo_t: {
          return EnviroSensorDemo::DESCRIPTION;
        } break;

        case IMUDemo_t: {
          return IMUSensorDemo::DESCRIPTION;
        } break;

        case AdafruitGPSDemo_t: {
          return AdafruitGPSDemo::DESCRIPTION;
        } break;

        case RadioClientDemo_t: {
          return RadioClientDemo::DESCRIPTION;
        } break;

        case RadioServerDemo_t: {
          return RadioServerDemo::DESCRIPTION;
        } break;

        case RadioWithGliderDemo_t: {
          return RadioWithGliderDemo::DESCRIPTION;
        } break;

        case SerialToRadioDemo_t: {
          return SerialToRadioDemo::DESCRIPTION;
        } break;

        case GroundStation_t: {
          return GroundStation::DESCRIPTION;
        } break;

        case ServoDriverDemo_t: {
          return ServoDriverDemo::DESCRIPTION;
        } break;

        case CompSystem_t:
        default:
          return "Competition System";
          break;
    }
  }
};
