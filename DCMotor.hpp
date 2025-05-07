#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

// Motor A (Left Motor)
#define ENA 6
#define IN1 8
#define IN2 7

// Motor B (Right Motor)
#define ENB 5
#define IN3 12
#define IN4 4

#define MOTOR_SPEED 90

class MotorDriver 
{
public:
    void setupMotors();
    void moveForward();
    void moveBackward();
    void moveStop();
    void turnRight();
    void turnLeft();
};

extern MotorDriver motor;

#endif
