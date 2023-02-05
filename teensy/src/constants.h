#include <Arduino.h>

#define MOTOR_VELOCITY_LIMIT 10.0 // rotations per second
#define MOTOR_CURRENT_LIMIT  20.0 // amps

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

const float m1 = 2.32f;    
const float m2 = 4.194f; 
const float I1 = 0.0784160f;
const float I2 = 0.0380256f;
const float mt = m1 + m2;
const float l1 = 0.26f;
const float l2 = 0.05f;
const float g  = 9.81f;
const float incline = 0.0f; 
const float k = 10.0f;
const float alpha = 360.0f/k/2.0f * M_PI/180.0f;
const float Kv = 0.13f;