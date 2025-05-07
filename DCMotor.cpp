#include "DCMotor.hpp"

MotorDriver motor;

void MotorDriver::setupMotors() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Serial.println("Motor set up");
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
}

void MotorDriver::moveForward() {
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Move Forward");
}

void MotorDriver::moveBackward() {
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Backward");
}

void MotorDriver::moveStop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("STOP!!!!!");
}

void MotorDriver::turnRight() {
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Right");
    delay(100);
}

void MotorDriver::turnLeft() {
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("left");
    delay(100);
}
