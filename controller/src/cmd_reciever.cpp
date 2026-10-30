#include "controller.hpp"
#include "motor.hpp"

#define AVERAGE_LENGTH 7

int main() {

    int speeds[AVERAGE_LENGTH];
    int index, nextSpeed, motorSum, motorSpeed, servoAngle;

    MotorController motor(SERVO_CHANNEL, MOTOR_CHANNEL); 
    motor.setMotorSpeed(MOTOR_START, true); 
    motor.setServoAngle(STRAIGHT);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.connect("tcp://localhost:5556"); // Connect to the sender's IP and port

    subscriber.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages

    // Init running averages
    for (int i = 0; i < AVERAGE_LENGTH; i++) {
        speeds[i] = MIN_THROTTLE;
    }
    motorSum = MIN_THROTTLE * AVERAGE_LENGTH;
    index = 0;

    for (int s = MOTOR_START; s <= MAX_THROTTLE; s++) {
        motor.setMotorSpeed(s, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    motor.setMotorSpeed(MIN_THROTTLE, true);
    std::cout << "Starting!" << std::endl;

    while (true) {
        zmq::message_t message;
        subscriber.recv(message, zmq::recv_flags::none);

        Command receivedCommand;
        memcpy(&receivedCommand, message.data(), sizeof(Command));

        servoAngle = (int) ((receivedCommand.angle / 2.0 + 0.5) * STEERING_RANGE + MIN_STEERING);
        nextSpeed = (int) (receivedCommand.speed * THROTTLE_RANGE + MIN_THROTTLE);

        motorSum -= speeds[index];
        speeds[index] = nextSpeed;
        motorSum += speeds[index];
        motorSpeed = motorSum / AVERAGE_LENGTH;

        std::cout << "Setting Motor Speed: " << motorSpeed << " Servo Angle: " << servoAngle << std::endl;
        motor.setMotorSpeed(motorSpeed, false);
        motor.setServoAngle(servoAngle);

        index = (index + 1) % AVERAGE_LENGTH;
    }

    return 0;
}