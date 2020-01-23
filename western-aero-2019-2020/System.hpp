/** \file System.hpp
 * @brief Contains all system base class related code
 */

/**
 * abstract class for defining how a system should be structured
 */

#pragma once

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"
#include "Imu.hpp"
#include "Radio.hpp"
#include "MockData.hpp"
#include "PitotTube.hpp"
#include "Enviro.hpp"
#include "GPS.hpp"
#include "src/Rfm95w/RH_RF95.h"
#include "Servo.hpp"


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
      TestSystem_t      = 0b00000000, // System for testing system select
      TesttxSerial_t    = 0b00000001, // System for sending messages based on protocol over serial
      ZTRDemo1Gnd_t     = 0b00000010, // System for first ZTR target demo
      PitotDemo_t       = 0b00000011, // System for testing the analog phidget pitot tube
      EnviroDemo_t      = 0b00000100, // System for testing the environment sensor
      IMUDemo_t         = 0b00000101, // System for testing the imu sensor
      AdafruitGPSDemo_t = 0b00000110, // System for testing the adafruit gps module
      CompSystem_t      = 0b00001111  // System for competition
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
        
//        case AdafruitGPSDemo_t: {
//          return new AdafruitGPSDemo();
//        } break;

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
        
//        case AdafruitGPSDemo_t: {
//          return AdafruitGPSDemo::DESCRIPTION;
//        } break;
        
        case CompSystem_t:
        default:
          return "Competition System";
          break;
    }
  }
};
