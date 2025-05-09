#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define POSITION_IN_ANGLES 180
#define MAX_DISTANCE 50
#define SERVO_PIN 11
#define SCAN_STEP 5
#define TOTAL_STEPS ((POSITION_IN_ANGLES / SCAN_STEP) + 1)

extern SemaphoreHandle_t xSerialSemaphore;

enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP,
    UNKNOWN
};

class Utility {
public:
    static void setup();
    static void loop();
    static void positionInDegree(int positionInDegrees);
    static void createTask();
    static void hcsr04ReadTask(void* pvParameters);
    static Direction getDirectionFromAngle(int angle);
    static Direction currentDirection;
    static int distanceAtAngle[TOTAL_STEPS];
    static int currentServoAngle;
 
};

#endif // UTILITY_HPP
