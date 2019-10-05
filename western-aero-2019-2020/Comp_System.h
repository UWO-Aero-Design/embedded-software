/*
 * implementation of a system that is used during competition
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Arduino.h"
#include "System.h"
#include "IMU_MPU9250.h"

class Comp_System : public System {
  public:
    Comp_System();
    ~Comp_System();
    void initSystem() override;
    void updateSystem() override;
  
  protected:

  
  private:
    const String type = String("This is a competition system");
    IMU_MPU9250 *imu;
  
};
