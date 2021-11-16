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
        
        enviro.begin();
    
        m_initialized = true;
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

//         if(millis() - last_update >= UPDATE_DELTA) {
          float temperature = enviro.readTemperature();
          float pressure = enviro.readPressure();
          float _altitude = enviro.readAltitude(zro_alt_pressure);
          float agl = _altitude - zero_altitude;
          if(agl < -10) agl = -10;
          float biased_value = agl;
          
          // check if error
          if(_altitude == -999 || temperature == -999 || pressure == -999) {
            return false;
          }
          
          m_data.altitude = (uint16_t)((agl * STRUCT_ALTITUDE_OFFSET));
          m_data.pressure = (uint16_t)(pressure * STRUCT_PRESSURE_OFFSET);
          m_data.temperature = (uint16_t)(temperature * STRUCT_TEMPERATURE_OFFSET);
          last_update = millis();
//         }
          
        return true;
    }

    void calibrate() {
      // Amount of barometer samples for calibration
      static const int SAMPLES = 25;
  
      float accumulated_pressure = 0.0f;
      for(int i = 0; i < SAMPLES; i++) {
        float alt = enviro.readAltitude(zero_al_pressure);
        accumulated_pressure += alt;
        delay(512); // Delay for update rate
      }
  
      zero_altitude = accumulated_pressure / SAMPLES;
      // Serial.println(zero_altitude);
      
    }

    float offset() { return zero_altitude; }

    static constexpr const float STRUCT_ALTITUDE_OFFSET = 100.0;
    static constexpr const float STRUCT_ALTITUDE_BIAS = 10.0;
    static constexpr const float STRUCT_PRESSURE_OFFSET = 100.0;
    static constexpr const float STRUCT_TEMPERATURE_OFFSET = 100.0;

private:
    
    // Track whether sensor has been initialized
    bool m_initialized = false;

    // An environment sensor
    Adafruit_BMP280 enviro;

    float raw_altitude = 0;
    float zero_altitude = 0;
    float zero_alt_pressure = enviro.readPressure();
    static constexpr const float ELEVATION_OFFSET = 0.0f;

    long last_update = 0;
    const int UPDATE_DELTA = 512;
};
