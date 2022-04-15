/** \file Enviro_Bmp280.hpp
 * @brief All code relating to the environment sensor
 */

/**
 * @brief Updated from previous code for MPL3115A2 sensor to new BMP280 sensor
 */

#pragma once

// Include for sensor interfaces
#include "src/aero-cpp-lib/include/Sensors.hpp"

#include "Wire.h"
#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
#include "Addresses.hpp"

/**
 * @brief Class for a enviornment sensor using the BMP280 chip
 * @details It is an I2C sensor capable of measuring pressure, humidity and temperature
 */
class AdafruitBMP280EnviroSensor : public aero::sensor::EnviroSensor {
public:
    /**
     * @brief Construct a new BMP280 Enviro Sensor object
     */
    AdafruitBMP280EnviroSensor(){}

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
        m_data.pressure = 0;
        
        m_initialized = m_bmp280.begin(Addresses::BMP280);
    
        return m_initialized;
    }

    /**
     * @brief Update sensor value by reading registers over I2C
     * 
     * @return true sensor value updated
     * @return false sensor read error
     */
    bool update() override {
        // If system is not initialized, return error
        if(!m_initialized) {
            return false;
        }

          m_data.temperature = m_bmp280.readTemperature();
          m_data.pressure = m_bmp280.readPressure();
          m_raw_altitude = m_bmp280.readAltitude(ZERO_PRESSURE);
          m_data.altitude = m_raw_altitude - m_zero_altitude;
          
          // check if error
          if(m_data.altitude == NAN || m_data.temperature == NAN || m_data.pressure == NAN) {
            return false;
          }
          
        return true;
    }

    bool calibrate() {
      // If system is not initialized, return error
        if(!m_initialized) {
            return false;
        }
      
      // Amount of barometer samples for calibration
      static constexpr const int SAMPLES = 200;
  
      float accumulated_pressure = 0.0f;
      for(int i = 0; i < SAMPLES; i++) {
        float alt = m_bmp280.readAltitude(ZERO_PRESSURE);
        accumulated_pressure += alt;
      }
  
      m_zero_altitude = accumulated_pressure / SAMPLES;

      return true;
      
    }

    float get_offset() { 
      return m_zero_altitude;
    }

private:

    Adafruit_BMP280 m_bmp280;
    // Track whether sensor has been initialized
    bool m_initialized = false;

    float m_pressure = 0;
    float m_temperature = 0;
    float m_altitude = 0;
    float m_raw_altitude = 0;
    float m_zero_altitude = 0;
    static constexpr const float ZERO_PRESSURE = 1013.25;
};
