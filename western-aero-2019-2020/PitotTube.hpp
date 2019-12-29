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
        float scaledVal = map(pitotVal, 0, 1023, 0, SCALED_VOLTAGE);
        float voltageVal = map(scaledVal, 0, SCALED_VOLTAGE, 0, PITOT_VOLTAGE);
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

    int m_pin;
    float m_pressure_kpa = 0.0f;

    // Values that need to be modified
    constexpr double R1 = 10000;
    constexpr double R2 = 10000*2;

    // No need to touch
    constexpr double PITOT_VOLTAGE = 5.00;
    constexpr double VOLTAGE_RATIO = R2/(R1+R2);
    constexpr double SCALED_VOLTAGE = VOLTAGE_RATIO*PITOT_VOLTAGE; 

};

#endif
