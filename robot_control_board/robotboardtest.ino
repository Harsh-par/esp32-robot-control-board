#include <esp_now.h>
#include <WiFi.h>

#ifdef I2C_INIT
#include <Wire.h>
#endif

#include "Define.h"
#include "Motor_Control.h"
#include "Servo_Control.h"
#include "Controller_Data.h"

volatile ControllerData Controller;

Motor_Control motor_A(pin_mSleep, pin_pMotorA, pin_nMotorA);
Motor_Control motor_B(pin_mSleep, pin_pMotorB, pin_nMotorB);
Motor_Control motor_C(pin_mSleep, pin_pMotorC, pin_nMotorC);
Motor_Control motor_D(pin_mSleep, pin_pMotorD, pin_nMotorD);

Servo_Control servo_A(pin_ServoA, pwm_Frequency, pwm_Resolution, pwm_ChannelA, min_PulseServo, max_PulseServo);
Servo_Control servo_B(pin_ServoB, pwm_Frequency, pwm_Resolution, pwm_ChannelB, min_PulseServo, max_PulseServo);
Servo_Control servo_C(pin_ServoC, pwm_Frequency, pwm_Resolution, pwm_ChannelC, min_PulseServo, max_PulseServo);

uint8_t current_AngleA = 0, current_AngleB = 0, current_AngleC = 0;
unsigned long previous_TimeServoA = 0, previous_TimeServoB = 0, previous_TimeServoC = 0;

unsigned long recieved_Time;

void TaskMotorControl(void *pvParameters);
void TaskServoControl(void *pvParameters);

void setup(){
  #ifdef DEBUG_MODE
  Serial.begin(BAUD_RATE);
  #endif

  pinMode(pin_VoltageDivider, INPUT);
  pinMode(pin_LedA, OUTPUT);
  pinMode(pin_LedB, OUTPUT);
  pinMode(pin_LedC, OUTPUT);

  xTaskCreatePinnedToCore(TaskMotorControl, "MotorControl", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoControl, "ServoControl", 4096, NULL, 1, NULL, 1);

  motor_A.Start();

  WiFi.mode(WIFI_STA);
  delay(100);
  esp_now_init();
  esp_now_register_recv_cb(ReceiveData);
  digitalWrite(pin_LedC, HIGH);

  #ifdef I2C_INIT
  Wire.setClock(I2C_CLK);               
  Wire.begin(pin_Sda, pin_Scl); 
  #endif
}

void loop(){
}

void TaskServoControl(void *pvParameters){
  while(1){
    unsigned long current_Time = millis();

    if(Controller.value_RJY > JOYSTICK_DEADZONE && (current_Time - previous_TimeServoA > SERVO_DELAY)){
      previous_TimeServoA = current_Time;
      current_AngleA = constrain(current_AngleA + 1, SERVO_MIN, SERVO_MAX);
      servo_A.SetAngle(current_AngleA);
    }
    else if(Controller.value_RJY < -JOYSTICK_DEADZONE && (current_Time - previous_TimeServoA > SERVO_DELAY)){
      previous_TimeServoA = current_Time;
      current_AngleA = constrain(current_AngleA - 1, SERVO_MIN, SERVO_MAX);
      servo_A.SetAngle(current_AngleA);
    }

    if(Controller.value_RJX > JOYSTICK_DEADZONE && (current_Time - previous_TimeServoC > SERVO_DELAY)){
      previous_TimeServoC = current_Time;
      current_AngleC = constrain(current_AngleC - 1, SERVO_MIN, SERVO_MAX);
      servo_C.SetAngle(current_AngleC);
    }
    else if(Controller.value_RJX < -JOYSTICK_DEADZONE && (current_Time - previous_TimeServoC > SERVO_DELAY)){
      previous_TimeServoC = current_Time;
      current_AngleC = constrain(current_AngleC + 1, SERVO_MIN, SERVO_MAX);
      servo_C.SetAngle(current_AngleC);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void TaskMotorControl(void *pvParameters){
  while(1){
    int joystick_Direction = MapJoystickToDirection(Controller.value_LJX, Controller.value_LJY); 

    switch(joystick_Direction){
    case DIRECTION_CENTER:
      motor_A.SetDirection(MOTOR_COAST);
      motor_B.SetDirection(MOTOR_COAST);
      motor_C.SetDirection(MOTOR_COAST);
      motor_D.SetDirection(MOTOR_COAST);
      break;
    case DIRECTION_UP:
    digitalWrite(pin_LedB, HIGH);
      motor_A.SetDirection(MOTOR_FORWARD);
      motor_B.SetDirection(MOTOR_FORWARD);
      motor_C.SetDirection(MOTOR_FORWARD);
      motor_D.SetDirection(MOTOR_FORWARD);
      break;
    case DIRECTION_DOWN:
    digitalWrite(pin_LedB, LOW);
      motor_A.SetDirection(MOTOR_REVERSE);
      motor_B.SetDirection(MOTOR_REVERSE);
      motor_C.SetDirection(MOTOR_REVERSE);
      motor_D.SetDirection(MOTOR_REVERSE);
      break;
    case DIRECTION_LEFT:
    digitalWrite(pin_LedA, HIGH);
      motor_A.SetDirection(MOTOR_FORWARD);
      motor_B.SetDirection(MOTOR_REVERSE);
      motor_C.SetDirection(MOTOR_FORWARD);
      motor_D.SetDirection(MOTOR_REVERSE);
      break;
    case DIRECTION_RIGHT:
    digitalWrite(pin_LedA, LOW);
      motor_A.SetDirection(MOTOR_REVERSE);
      motor_B.SetDirection(MOTOR_FORWARD);
      motor_C.SetDirection(MOTOR_REVERSE);
      motor_D.SetDirection(MOTOR_FORWARD);
      break;
    default:
      motor_A.SetDirection(MOTOR_COAST);
      motor_B.SetDirection(MOTOR_COAST);
      motor_C.SetDirection(MOTOR_COAST);
      motor_D.SetDirection(MOTOR_COAST);
      break;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

int MapJoystickToDirection(int joystick_X, int joystick_Y) {
  if (abs(joystick_X) < JOYSTICK_DEADZONE && abs(joystick_Y) < JOYSTICK_DEADZONE) return DIRECTION_CENTER;
  if (joystick_Y > 0 && abs(joystick_Y) > abs(joystick_X)) return DIRECTION_UP;
  if (joystick_Y < 0 && abs(joystick_Y) > abs(joystick_X)) return DIRECTION_DOWN;
  if (joystick_X > 0 && abs(joystick_X) > abs(joystick_Y)) return DIRECTION_RIGHT;
  if (joystick_X < 0 && abs(joystick_X) > abs(joystick_Y)) return DIRECTION_LEFT;
  return DIRECTION_CENTER;
}

void ReceiveData(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int data_len){
  memcpy((void*)&Controller, data, sizeof(Controller));
  recieved_Time = millis();
}