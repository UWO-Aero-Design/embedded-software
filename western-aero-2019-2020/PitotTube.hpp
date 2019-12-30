#ifndef _PITOT_TUBE_H_
#define _PITOT_TUBE_H_

// Include for sensor interfaces; pitot tube needed
#include "src/aero-cpp-lib/include/Sensors.hpp"

/**
 * @brief Class for a differential pressure made by Phidget
 * 
 * @details It is an analog 0 -> 5.0 V sensor with a range of -7 to 7 kPa centered around 2.5 V
 *          https://www.phidgets.com/?tier=3&catid=7&pcid=5&prodid=110
 * 
 * @note    This class is made for a Teensy3.2/3.5 that uses a 0 -> 3.3 V ADC
 *          Therefore, it is assumed a voltage divider is being used cause the Phidget sensor uses a 5 V range 
 */
class PhidgetPitotTube : public aero::sensor::Pitot {
public:

    /**
     * @brief Construct a new Phidget Pitot Tube object
     * 
     * @param pin Analog pin for the Phidget sensor
     */
    PhidgetPitotTube(int pin) : m_pin(pin) {}

    /**
     * @brief Initialize sensor object
     * 
     * @return true If init succeeded
     * @return false If init failed
     */
    bool init() override {
        // Initialize data member 
        m_data.differential_pressure = 0;
        
        // Read the pin and make sure it returns a value that makes sense
        int pitot_val = analogRead(m_pin);
        
        // Assumption that a valid non-floating value is above 100. Also, safe assumption that we wont be reading 100 on bootup
        if(pitot_val > NON_FLOATING_READING) {
            m_initialized = true;
        } 

        m_initialized true;
    }

    /**
     * @brief Update sensor value by reading voltage on pin
     * 
     * @return true sensor value updated
     * @return false sensor value not updated
     */
    bool update() override {
        // If system is not initialized, return error
        if(!m_initialized) {
            return false;
        }

        // Read in a 0-1023 analog signal that corresponds to 0 to SCALED_VOLTAGE
        int pitot_val = analogRead(m_pin);

        // Convert the analog reading to the actual voltage based on analog resolution
        double scaledVal = mapf(pitotVal, 0, 1023, 0, SCALED_VOLTAGE);

        // Convert the voltage to the 0 to 5 range of the pitot tube
        double voltageVal = mapf(scaledVal, 0, SCALED_VOLTAGE, 0, PITOT_VOLTAGE);

        // Calculate the voltage ratio of the sensor reading to calculate pressure
        double voltageRatio = voltageVal/PITOT_VOLTAGE;

        // Pressure calculations
        m_pressure_kpa = tokPa(voltageRatio);
        m_data.differential_pressure = (int)(m_pressure_kpa*DIF_PRES_SCALAR);
        
        return true;
    }

    /**
     * @brief Get differential pressure
     * 
     * @return double differential pressure in kPa
     */
    double pressure() const {
        return m_pressure_kpa;
    }

private:
    // Helper function to convert voltage ratio of sensor to differential pressure with kPa as units
        // Based on sensor spec
    double tokPa(double voltRatio) {
        return ( ( voltRatio*17.5 ) - 8.75 );
    }

    // Helper function to convert voltage ratio of sensor to differential pressure with psi as units
        // Based on sensor spec
    double topsi(double voltRatio) {
        return ( ( voltRatio*2.538 ) - 1.269 );
    }

    // Helper function to map float values from one range of values to another
    double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    // Analog pin to read
    int m_pin = 0;
    
    // Last read differential pressure value
    double m_pressure_kpa = 0.0f;
    
    // Track whether sensor has been initialized
    bool m_initialized = false;

    // Voltage divider resistors to calculate scaled voltage
    constexpr static double R1 = 10000;
    constexpr static double R2 = 10000*2;
    constexpr static double VOLTAGE_RATIO = R2/(R1+R2);
    // Pitot voltage has a 0 to 5 V output voltage range
    constexpr static double PITOT_VOLTAGE = 5.00;
    // Calculate scaled voltage range. Goal is to convert the pitot voltage into a 0 -> 3.3 V range to work with the Teensy ADC
    constexpr static double SCALED_VOLTAGE = VOLTAGE_RATIO*PITOT_VOLTAGE; 

    // Min value that is assumed not to be floating
    constexpr static int NON_FLOATING_READING = 100;

    // Value that scales the differential pressure reading to the message protocol
    constexpr static int DIF_PRES_SCALAR = 1000;
};

#endif
