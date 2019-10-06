#include "System.h"
#include "Wire.h"

CompSystem::CompSystem() {
  
}

CompSystem::~CompSystem() {
  
}


void CompSystem::init() {
  Wire.begin();
  imu = new ImuMpu9250();
}

void CompSystem::update() {
  Serial.println(type);
  Serial.println(imu->type);
}
