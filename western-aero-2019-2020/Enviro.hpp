/** \file Enviro.hpp
 * @brief All code relating to the environment sensor
 */


#pragma once

// Include for sensor interfaces
#include "src/aero-cpp-lib/include/Sensors.hpp"

#include "Wire.h"
#include "src/MPL3115A2/MPL3115A2.h"

/**
 * @brief Class for a enviornment sensor using the MPL3115A2 chip
 * @details It is an I2C sensor capable of measuring pressure, humidity and temperature
 */
class Mpl3115a2EnviroSensor : public aero::sensor::EnviroSensor {
public:

    /**
     * @brief Construct a new MPL3115A2 Enviro Sensor object
     */
    Mpl3115a2EnviroSensor(){}

    /**
     * @brief Initialize sensor object
     * 
     * @return true If init succeeded
     * @return false If init failed
     */
    bool init() override {
        // Initialize data member 
        m_data.altitude = 0;
        m_data.temperature = 0;
        
        enviro.begin();
        enviro.setModeAltimeter();
        enviro.setOversampleRate(7);
        enviro.enableEventFlags();
        m_initialized = true;
        return true;
    }

    /**
     * @brief Update sensor value by reading registers over I2C
     * 
     * @return true sensor value updated
     * @return false sensor value not updated
     */
    bool update() override {
        // If system is not initialized, return error
        if(!m_initialized) {
            return false;
        }
        
        float altitude = enviro.readAltitude();
        float temperature = enviro.readTemp();
        
        // check if error
        if(altitude == -999 || temperature == -999) {
          return false;
        }
        
        // retreive data from sensor
        m_data.altitude = (uint16_t)(altitude * ALTITUDE_OFFSET);
        m_data.temperature = (uint16_t)(temperature * TEMPERATURE_OFFSET);
          
        return true;
    }

    static constexpr const float ALTITUDE_OFFSET = 100.0;
    static constexpr const float TEMPERATURE_OFFSET = 100.0;

private:
    
    // Track whether sensor has been initialized
    bool m_initialized = false;

    // An environment sensor
    MPL3115A2 enviro;

};
