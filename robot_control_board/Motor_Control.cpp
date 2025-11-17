#include "esp32-hal-gpio.h"
#include "Motor_Control.h"
#include <Arduino.h>
#include "Define.h"

Motor_Control::Motor_Control(uint8_t pin_mSleep, uint8_t pin_mPlus, uint8_t pin_mMinus): pin_mSleep(pin_mSleep), pin_mPlus(pin_mPlus), pin_mMinus(pin_mMinus){
  pinMode(pin_mSleep, OUTPUT);
  pinMode(pin_mPlus, OUTPUT);
  pinMode(pin_mMinus, OUTPUT);
}

void Motor_Control::SetDirection(int State){
  switch(State){
  case MOTOR_FORWARD:
    digitalWrite(pin_mSleep, HIGH);
    digitalWrite(pin_mPlus,  HIGH);
    digitalWrite(pin_mMinus, LOW);
    break;
  case MOTOR_COAST:
    digitalWrite(pin_mSleep, HIGH);
    digitalWrite(pin_mPlus,  HIGH);
    digitalWrite(pin_mMinus, HIGH);
    break;
  case MOTOR_REVERSE:
    digitalWrite(pin_mSleep, HIGH);
    digitalWrite(pin_mPlus,  LOW);
    digitalWrite(pin_mMinus, HIGH);
    break;
  case MOTOR_DECAY:
    digitalWrite(pin_mSleep, HIGH);
    digitalWrite(pin_mPlus,  LOW);
    digitalWrite(pin_mMinus, LOW);
    break;
  default:
    break;
  }
}

void Motor_Control::Start(void){
  digitalWrite(pin_mSleep, HIGH);
}

void Motor_Control::Sleep(void){
  digitalWrite(pin_mSleep, LOW);
  digitalWrite(pin_mPlus,  LOW);
  digitalWrite(pin_mMinus, LOW);
}