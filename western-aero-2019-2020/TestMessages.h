#pragma once

#include "Arduino.h"
#include "src/aero/aero-cpp-lib/include/Message.hpp"

int16_t random_int16(void) {
    return static_cast<int16_t>(random(-32768, 32768));
}

aero::def::IMU_t random_imu(void) {
    aero::def::IMU_t imu {
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16(),
        random_int16()
    };
    return imu;
}
