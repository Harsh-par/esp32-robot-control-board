#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class Motor_Control{
  public:
    Motor_Control(uint8_t pin_mSleep, uint8_t pin_mPlus, uint8_t pin_mMinus);
    void SetDirection(int State);
    void Start(void);
    void Sleep(void);
    
  private:
    uint8_t pin_mSleep, pin_mPlus, pin_mMinus;
};

#endif
