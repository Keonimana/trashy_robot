#include "motor.hpp" 
#include <iostream>

MotorController::MotorController(uint8_t servo_channel, uint8_t motor_channel) {
    pca = new PCA9685(1, DEVICE_ADDRESS); 
    pca->setPWMFreq(50); 
    servo = servo_channel; 
    motor = motor_channel; 
    m_speed = MOTOR_START; 
    s_angle = STRAIGHT; 
}

MotorController::~MotorController() {
    pca->setPWM(motor, 0); 
    pca->setPWM(servo, 0); 
    delete pca; 
    printf("Freeing MotorController Controller\n"); 
}

void MotorController::setMotorSpeed(int speed, bool force) {
    if (force) {
        m_speed = speed;
    } else {
        m_speed = (speed > MAX_THROTTLE) ? MAX_THROTTLE : speed;
        m_speed = (m_speed < MIN_THROTTLE) ? MIN_THROTTLE : m_speed;
    }
    pca->setPWM(motor, m_speed); 
}

void MotorController::setServoAngle(int angle) {
    s_angle = (angle > MAX_STEERING) ? MAX_STEERING : angle;
    s_angle = (s_angle < MIN_STEERING) ? MIN_STEERING : s_angle;
    pca->setPWM(servo, s_angle); 
}

int MotorController::getMotorSpeed() {
   return pca->getPWM(motor); 
}

int MotorController::getServoAngle() {
    return pca->getPWM(servo); 
}

int MotorController::getPWMFrequency(int channel) {
    return pca->getPWM(channel); 
}

int MotorController::stepForwards() {
    m_speed += INCREMENT; 
    m_speed = (m_speed > MAX_THROTTLE) ? MAX_THROTTLE : m_speed;

    pca->setPWM(motor, m_speed); 
    return 0;
}

int MotorController::stepBackwards() {
    m_speed -= INCREMENT; 
    m_speed = (m_speed < MOTOR_START) ? MOTOR_START : m_speed;

    pca->setPWM(motor, m_speed); 
    return 0;
}

int MotorController::stepLeft() {
    s_angle -= INCREMENT; 
    s_angle = (s_angle < MIN_STEERING) ? MIN_STEERING : s_angle;

    pca->setPWM(servo, s_angle); 
    return s_angle;
}

int MotorController::stepRight() {
    s_angle += INCREMENT; 
    s_angle = (s_angle > MAX_STEERING) ? MAX_STEERING : s_angle;

    pca->setPWM(servo, s_angle); 
    return s_angle; 
}





