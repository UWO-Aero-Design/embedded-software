#ifndef PITOT_TUBE_H
#define PITOT_TUBE_H

#include "src/aero-cpp-lib/include/Sensors.hpp"
class PhidgetPitotTube : public aero::sensor::Pitot {
public:
    PhidgetPitotTube(int pin) {
        m_pin = pin;
    }

    bool init() override {
        // Init function of the sensor
        // Maybe read value and make sure its not floating?
    }

    bool update() override {
        // Read in a 0-1023 analog signal that corresponds to 0 to SCALED_VOLTAGE
        int pitotVal = analogRead(m_pin);

        // Convert pitot value to voltage (0 to SCALED_VOLTAGE)
        float scaledVal = mapf(pitotVal, 0, 1023, 0, SCALED_VOLTAGE);
        float voltageVal = mapf(scaledVal, 0, SCALED_VOLTAGE, 0, PITOT_VOLTAGE);
        float voltageRatio = voltageVal/PITOT_VOLTAGE;

        // Now we apply the pitot equation
        m_pressure_kpa = tokPa(voltageRatio);
        // Apply converion rate 
        m_data.differential_pressure = (int)(m_pressure_kpa*1000);

        // PSI
        float psi = topsi(voltageRatio);
        
    }

    // Data returns the message data
    // data();

    // In kPa or Pa
    double pressure() const {
        return m_pressure_kpa;
    }

private:
    double tokPa(double voltRatio) {
        return ( ( voltRatio*17.5 ) - 8.75 );
    }

    double topsi(double voltRatio) {
        return ( ( voltRatio*2.538 ) - 1.269 );
    }

    double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    int m_pin;
    float m_pressure_kpa = 0.0f;

    // Values that need to be modified
    constexpr static double R1 = 10000;
    constexpr static double R2 = 10000*2;

    // No need to touch
    constexpr static double PITOT_VOLTAGE = 5.00;
    constexpr static double VOLTAGE_RATIO = R2/(R1+R2);
    constexpr static double SCALED_VOLTAGE = VOLTAGE_RATIO*PITOT_VOLTAGE; 

};

#endif
