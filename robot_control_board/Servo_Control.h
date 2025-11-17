#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

class Servo_Control{
  public:
    Servo_Control(uint8_t pin_Servo, uint8_t pwm_Frequency, uint8_t pwm_Resolution, uint8_t pwm_Channel, uint16_t minimum_Pulse, uint16_t maximum_Pulse);
    void SetAngle(uint8_t Angle);
    
  private:
    uint8_t  pin_Servo;
    uint8_t  pwm_Frequency;
    uint8_t  pwm_Resolution;
    uint8_t  pwm_Channel;
    float    pwm_Factor;
    uint16_t minimum_Pulse;
    uint16_t maximum_Pulse;

    static const uint8_t minimum_Angle = 0;
    static const uint8_t maximum_Angle = 180;
};

#endif