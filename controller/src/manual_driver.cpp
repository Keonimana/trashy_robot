#include <unistd.h> 
#include "motor.hpp"
#include <ncurses.h> 

using namespace std; 

void loop(MotorController* motor);
void setup(MotorController* motor);
void cleanup(MotorController* motor);

int main(int argc, char** argv) {
    // Create motor object
    MotorController motor(SERVO_CHANNEL, MOTOR_CHANNEL); 

    setup(&motor);
    loop(&motor);
    cleanup(&motor);

    return 0;
}

void setup(MotorController* motor) { 
    // Set initial motor conditions
    motor->setMotorSpeed(NEUTRAL, true); 
    motor->setServoAngle(STRAIGHT);

    // Setup curses
    initscr();
}

void cleanup(MotorController* motor) {
    // Ensure car stops moving
    motor->setMotorSpeed(NEUTRAL, true);
    motor->setServoAngle(STRAIGHT);

    // Teardown curses
    endwin();
}

void loop(MotorController* motor) {

    char cmd;
    int motor_speed;

    // Continuously loop through taking commands
    do {
        cmd = getch();
        refresh();  

        switch(cmd) {
            case 'w': 
                motor->stepForwards(); 
                break; 
            case 's': 
                motor->stepBackwards(); 
                break; 
            case 'a': 
                motor->stepLeft(); 
                break; 
            case 'd': 
                motor->stepRight(); 
                break; 
            case 'q':
                printw("Exiting Loop");
                break;
            default:
                printw("Command not recognized\n");
                break;
        }
        motor_speed = motor->getMotorSpeed(); 
        printw("S: %x  ", motor_speed); 
    }
    while (cmd != 'q');
} 
