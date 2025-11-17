#include "Servo_Control.h"
#include <Arduino.h>

Servo_Control::Servo_Control(uint8_t pin_Servo, uint8_t pwm_Frequency, uint8_t pwm_Resolution, uint8_t pwm_Channel, uint16_t minimum_Pulse, uint16_t maximum_Pulse) : 
  pin_Servo(pin_Servo), pwm_Frequency(pwm_Frequency), pwm_Resolution(pwm_Resolution), pwm_Channel(pwm_Channel), minimum_Pulse(minimum_Pulse), maximum_Pulse(maximum_Pulse){
  
  pwm_Factor = 1.0 * (1 << pwm_Resolution) / (1000000.0 / pwm_Frequency); 
  ledcAttachChannel(pin_Servo, pwm_Frequency, pwm_Resolution, pwm_Channel);
}

void Servo_Control::SetAngle(uint8_t Angle){
  Angle = constrain(Angle, minimum_Angle, maximum_Angle);

  uint16_t pwm_Pulse = map(Angle, minimum_Angle, maximum_Angle, minimum_Pulse, maximum_Pulse);
  uint32_t pwm_Input = (uint32_t) (pwm_Factor * pwm_Pulse);

  ledcWrite(pin_Servo, pwm_Input);
}
