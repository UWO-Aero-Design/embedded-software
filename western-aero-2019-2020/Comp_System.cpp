#include "Comp_System.h"

Comp_System::Comp_System() {
  
}

Comp_System::~Comp_System() {
  
}


void Comp_System::initSystem() {
  imu = new IMU_MPU9250();
}

void Comp_System::updateSystem() {
  Serial.println(type);
  Serial.println(imu->type);
}
