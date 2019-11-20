/*
 * implementation of a system that is used for a full system test
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Arduino.h"
#include "src/MPU9250/MPU9250.h"

class ImuMpu9250 {
  public:
    ImuMpu9250();
    ~ImuMpu9250();
    void init();
    void update();
    String type = String("This is an IMU");
};

class TestImuMpu9250 {
  public:
    TestImuMpu9250();
    ~TestImuMpu9250();
    void init();
    void update();
    String type = String("This is a test IMU");
};
