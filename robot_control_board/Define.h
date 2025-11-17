#ifndef DEFINE_H
#define DEFINE_H

#define DEBUG_MODE      1
//#define I2C_INIT  1

#define BAUD_RATE 115200
#define I2C_CLK   400000

#define ESPNOW_PASSWORD "RX_1_Password"
#define ESPNOW_NETWORK  "RX_1"
#define ESPNOW_CHANNEL  1

#define SERVO_DELAY 15
#define SERVO_MAX   179
#define SERVO_MIN   0 

#define JOYSTICK_DEADZONE 5

#define DIRECTION_CENTER  0
#define DIRECTION_UP      1
#define DIRECTION_DOWN    2
#define DIRECTION_LEFT    3
#define DIRECTION_RIGHT   4

#define MOTOR_STOP        0
#define MOTOR_FORWARD     1
#define MOTOR_REVERSE     2
#define MOTOR_COAST       3
#define MOTOR_DECAY       4

//PWM Macros
constexpr uint16_t max_PulseServo = 2500;
constexpr uint16_t min_PulseServo = 500;
constexpr uint8_t  pwm_Frequency  = 50;
constexpr uint8_t  pwm_Resolution = 16;
constexpr uint8_t  pwm_ChannelA    = 0;
constexpr uint8_t  pwm_ChannelB    = 1;
constexpr uint8_t  pwm_ChannelC    = 2;


//Pins for Motors
constexpr uint8_t pin_pMotorA = 19, pin_pMotorB = 22, pin_pMotorC = 25, pin_pMotorD = 27;
constexpr uint8_t pin_nMotorA = 21, pin_nMotorB = 23, pin_nMotorC = 32, pin_nMotorD = 26;
constexpr uint8_t pin_mSleep = 33;         

//Pins for Servos
constexpr uint8_t pin_ServoA = 18, pin_ServoB = 17, pin_ServoC = 16;

//Pins for Led's, I2C, 
constexpr uint8_t pin_Sda = 4, pin_Scl = 5, pin_LedA = 14, pin_LedB = 12, pin_LedC = 2; 

//Pins for VoltageDivider
constexpr uint8_t pin_VoltageDivider = 35;

#endif