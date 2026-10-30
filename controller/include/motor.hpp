#pragma once
#include <cstdint>
#include <stdlib.h> 
#include <stdio.h> 
#include "PCA9685.h"

// Important Constants
#define MOTOR_START 0X0cc
#define MAX_THROTTLE 0x125
#define MIN_THROTTLE 0X108
#define MIN_STEERING 0x0d5
#define MAX_STEERING 0x148
#define THROTTLE_RANGE (MAX_THROTTLE - MIN_THROTTLE)
#define STEERING_RANGE (MAX_STEERING - MIN_STEERING)
#define FREQUENCY 50 
#define INCREMENT 0X1 
#define NEUTRAL MIN_THROTTLE
#define STRAIGHT (MAX_STEERING + MIN_STEERING) / 2

// Device State
#define DEVICE_ADDRESS 0x40 // Default address for i2c driver
#define SERVO_CHANNEL 16
#define MOTOR_CHANNEL 15

// Class to manipulate rc car motor speed. Stores PWM values at 12 bit resolution
class MotorController{
    public: 
        // Controller Object to change Motor PWM
        MotorController(uint8_t, uint8_t); // Takes in Servo and ESC channels
        ~MotorController(); 
        void setMotorSpeed(int speed, bool force); 
        void setServoAngle(int angle); 
        int stepForwards(); 
        int stepBackwards(); 
        int stepRight(); 
        int stepLeft(); 
        int getMotorSpeed(); 
        int getServoAngle(); 
        int getPWMFrequency(int); 

    private: 
        // Class variables
        PCA9685* pca; // Motor Driver
        int motor; 
        int servo; 
        int m_speed; 
        int s_angle; 
}; 
