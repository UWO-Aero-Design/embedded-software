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

    imu.yaw = 0xff;
    return imu;
}

aero::def::Pitot_t random_pitot(void) {
    aero::def::Pitot_t pitot {
        0xFFFF
    };
    return pitot;
}
