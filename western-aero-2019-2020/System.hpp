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
#include "Radio_Rfm95w.hpp"
#include "MockData.hpp"
#include "PitotTube.hpp"
#include "Enviro_Bmp280.hpp"
#include "Enviro_Mpl3115a2.hpp"
#include "GPS.hpp"
#include "src/Rfm95w/RH_RF95.h"
#include "Servo.hpp"
#include "DropAlgo.hpp"

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
    typedef enum {
      //Binary bits assigned according to 3 DIP Switches (2^3 possible combinations)
      PitotDemo_t           = 0b00000000, // System for testing the pitot tube
      EnviroDemo_t          = 0b00000001, // System for testing the environment sensor
      AdafruitGPSDemo_t     = 0b00000010, // System for testing the adafruit gps module
      RadioClientDemo_t     = 0b00000011, // System for testing the radio in client mode
      RadioServerDemo_t     = 0b00000101, // System for testing the radio in server mode
      IMUDemo_t             = 0b00000110, // System for testing the imu sensor
      CompSystem_t          = 0b00000111  // System for competition      
    } SystemType;

     /**
     * @brief Used to get a system object from a valid system type
     *
     * @param type Type of system to return (SystemType)
     * @return System pointer of the correct type
    */
    static System *select(SystemType type) {
      switch(type) {
        case PitotDemo_t: {
          return new PitotTubeDemo();
        } break;

        case EnviroDemo_t: {
          return new EnviroSensorDemo();
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

        case IMUDemo_t: {
          return new IMUSensorDemo();
        } break;

        case CompSystem_t: {
          return new CompSystem();
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
    static String get_description(SystemType type) {
      switch(type) {
        case PitotDemo_t: {
          return PitotTubeDemo::DESCRIPTION;
        } break;

        case EnviroDemo_t: {
          return EnviroSensorDemo::DESCRIPTION;
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

        case IMUDemo_t: {
          return IMUSensorDemo::DESCRIPTION;
        } break;
        
        case CompSystem_t:
        default:
          return "Competition System";
          break;
    }
  }
};
