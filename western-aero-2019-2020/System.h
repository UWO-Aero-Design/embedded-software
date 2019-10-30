/*
 * abstract class for defining how a system should be structured
 */

#pragma once

class System {
  public:
    System(){}
    virtual ~System(){}
    virtual void initSystem() = 0;
    virtual void updateSystem() = 0;
  
  protected:

  
  private:
  
  
};
