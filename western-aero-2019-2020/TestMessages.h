#pragma once

#include "Arduino.h"
#include "src/aero/aero-cpp-lib/include/Message.hpp"

int16_t random_int16(void) {
    return static_cast<int16_t>(random(-32768, 32768));
}

aero::def::IMU_t random_imu(void) {
    aero::def::IMU_t imu {
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x09,
        0x0A,
        0x0B,
        0x0C
    };

    return imu;
}

aero::def::Pitot_t random_pitot(void) {
    aero::def::Pitot_t pitot {
        0x0000
    };
    return pitot;
}

aero::def::GPS_t test_gps(void) {
    aero::def::GPS_t gps;
    // int32_t
    gps.lat = 0xABCDEFAB;
    // int32_t
    gps.lon = 0xCDEFABCD;
    // uint16_t
    gps.speed = 0xEFAB;
    // uint8_t
    gps.satellites = 0xCD;
    // uint32_t
    gps.time = 0xEFABCDEF;
    // uint32_t
    gps.date = 0xABCDEFAB;

    return gps;
}

aero::def::Enviro_t test_enviro(void) {
    aero::def::Enviro_t enviro;
    // uint16_t for all
    enviro.pressure = 0xABCD;
    enviro.humidity = 0xEFAB;
    enviro.temperature = 0xCDEF;

    return enviro;
}

aero::def::Battery_t test_battery(void) {
    aero::def::Battery_t batt;
    // uint16_t for all
    batt.voltage = 0xABCD;
    batt.current = 0xEFAB;

    return batt;
}

aero::def::SystemConfig_t test_system(void) {
    aero::def::SystemConfig_t system;
    // Empty struct
    return system;
}

aero::def::Status_t test_status(void) {
    aero::def::Status_t status;
    // int16_t
    status.rssi = 0xABCD;
    // uint32_t
    status.state = 0xEFABCDEF;

    return status;
}

aero::def::Servos_t test_servos(void) {
    aero::def::Servos_t servos;
    // All uint32
    servos.servo0 = 0xABCDEFAB;
    servos.servo1 = 0xCDEFABCD;
    servos.servo2 = 0xABCDEFAB;
    servos.servo3 = 0xCDEFABCD;
    servos.servo4 = 0xABCDEFAB;
    servos.servo5 = 0xCDEFABCD;
    servos.servo6 = 0xABCDEFAB;
    servos.servo7 = 0xCDEFABCD;
    servos.servo8 = 0xABCDEFAB;
    servos.servo9 = 0xCDEFABCD;
    servos.servo10 = 0xABCDEFAB;
    servos.servo11 = 0xCDEFABCD;
    servos.servo12 = 0xABCDEFAB;
    servos.servo13 = 0xCDEFABCD;
    servos.servo14 = 0xABCDEFAB;
    servos.servo15 = 0xCDEFABCD;

    return servos;
}

aero::def::AirData_t test_airdata(void) {
    aero::def::AirData_t airdata;
    // All uint32
    airdata.ias = 0xABCDEFAB;
    airdata.eas = 0xCDEFABCD;
    airdata.tas = 0xABCDEFAB;
    airdata.agl = 0xCDEFABCD;
    airdata.pressure_alt = 0xABCDEFAB;
    airdata.msl = 0xCDEFABCD;
    airdata.density_alt = 0xABCDEFAB;
    airdata.approx_temp = 0xCDEFABCD;
    airdata.density = 0xABCDEFAB;
    
    return airdata;
}

aero::def::Commands_t test_commands(void) {
    aero::def::Commands_t commands;
    // uint8_t
    commands.drop = 0xAB;
    // uint16_t
    commands.servos = 0xCDEF;
    // uint8_t
    commands.pitch = 0xAB;

    return commands;
}

aero::def::DropAlgo_t test_dropalgo(void) {
    aero::def::DropAlgo_t dropalgo;
    // int16_t
    dropalgo.heading = 0xABCD;
    // uint16_t
    dropalgo.distance = 0xEFAB;

    return dropalgo;
}