/*
 * abstract class for defining how a sensor should be structured
 */

#pragma once

class Sensor {
  public:
    Sensor(){}
    virtual ~Sensor(){}
    virtual void initSensor() = 0;
    virtual void updateSensor() = 0;
  
  protected:

  
  private:
  
  
};
