/** \file Enviro.hpp
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
        
        m_initialized = bmp280.begin();
    
        return m_initialized;
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

          temperature = bmp280.readTemperature();
          pressure = bmp280.readPressure();
          raw_altitude = bmp280.readAltitude(ZERO_PRESSURE);
          altitude = raw_altitude - zero_altitude;
          
          // check if error
          if(altitude == NAN || temperature == NAN || pressure == NAN) {
            return false;
          }
          
          m_data.altitude = (uint16_t)((altitude * STRUCT_ALTITUDE_OFFSET));
          m_data.pressure = (uint16_t)(pressure * STRUCT_PRESSURE_OFFSET);
          m_data.temperature = (uint16_t)(temperature * STRUCT_TEMPERATURE_OFFSET);
          
        return true;
    }

    bool calibrate() {
      // If system is not initialized, return error
        if(!m_initialized) {
            return false;
        }
      
      // Amount of barometer samples for calibration
      static constexpr const int SAMPLES = 3;
  
      float accumulated_pressure = 0.0f;
      for(int i = 0; i < SAMPLES; i++) {
        float alt = bmp280.readAltitude(ZERO_PRESSURE);
        accumulated_pressure += alt;
        delay(512); // Delay for update rate
      }
  
      zero_altitude = accumulated_pressure / SAMPLES;

      return true;
      
    }

    float offset() { return zero_altitude; }

    static constexpr const float STRUCT_ALTITUDE_OFFSET = 100.0;
    static constexpr const float STRUCT_ALTITUDE_BIAS = 10.0;
    static constexpr const float STRUCT_PRESSURE_OFFSET = 100.0;
    static constexpr const float STRUCT_TEMPERATURE_OFFSET = 100.0;

private:
    
    // Track whether sensor has been initialized
    bool m_initialized = false;

    Adafruit_BMP280 bmp280;

    float pressure = 0;
    float temperature = 0;
    float altitude = 0;
    float raw_altitude = 0;
    float zero_altitude = 0;
    static constexpr const float ZERO_PRESSURE = 1013.25;
};
