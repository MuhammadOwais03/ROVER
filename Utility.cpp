#include "Utility.hpp"
#include <HCSR04.h>
#include <Servo.h>
#include "DCMotor.hpp"
// #include "Motor.hpp"

static byte triggerPin = 2;
static byte echoPin = 3;
static Servo myservo;

#define IR_SENSOR_PIN 13

SemaphoreHandle_t xSerialSemaphore = NULL;
Direction Utility::currentDirection = UNKNOWN;
int Utility::distanceAtAngle[TOTAL_STEPS] = {0};

UltraSonicDistanceSensor sensor(triggerPin, echoPin);

void Utility::setup() {
    Serial.begin(9600);
    myservo.attach(SERVO_PIN);

    if (xSerialSemaphore == NULL) {
        xSerialSemaphore = xSemaphoreCreateMutex();
        if ((xSerialSemaphore) != NULL)
            xSemaphoreGive(xSerialSemaphore);
    }

    // MotorDriver::setupMotors();
}

void Utility::loop() {
    // This can be empty if task handles logic
}

void Utility::positionInDegree(int positionInDegrees) {
    myservo.write(positionInDegrees);
}

Direction Utility::getDirectionFromAngle(int angle) {
    if (angle > 90)
        return LEFT;
    else if (angle < 90)
        return RIGHT;
    return UNKNOWN;
}

void Utility::createTask() {
    xTaskCreate(
        hcsr04ReadTask,
        "hcsr04ReadTask",
        256,
        NULL,
        1,
        NULL
    );
}

void Utility::hcsr04ReadTask(void* pvParameters) 
{
    Utility::setup(); 
    //  motor.setupMotors();
    for (;;) 
    {

         int irValue = digitalRead(IR_SENSOR_PIN);
         Serial.println("IR Value");
         if (irValue == HIGH)  // Obstacle detected via IR
        {
            Utility::currentDirection = STOP;
            // motor.moveStop();
            vTaskDelay(pdMS_TO_TICKS(500));

            Utility::currentDirection = BACKWARD;
            // motor.moveStop();
            vTaskDelay(pdMS_TO_TICKS(500));

            Utility::currentDirection = STOP;
            vTaskDelay(pdMS_TO_TICKS(500));

            int maxDistance = 0;
            int maxAngle = 0;

            for (int angle = 0; angle <= 180; angle += SCAN_STEP) {
                Utility::positionInDegree(angle);
                vTaskDelay(pdMS_TO_TICKS(1));

                double distance = sensor.measureDistanceCm();
                int index = angle / SCAN_STEP;

                Utility::distanceAtAngle[index] = distance;

                Direction dir = getDirectionFromAngle(angle);
                if (distance > maxDistance && dir != FORWARD) {
                    maxDistance = distance;
                    maxAngle = angle;
                }
            }

            Utility::currentDirection = getDirectionFromAngle(maxAngle);
            Utility::positionInDegree(90);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
                Serial.print("IR Triggered | Max Distance: ");
                Serial.print(maxDistance);
                Serial.print(" cm at angle: ");
                Serial.println(maxAngle);
                Serial.print("Chosen Direction (excluding FORWARD): ");
                Serial.println(
                    currentDirection == LEFT ? "LEFT" :
                    currentDirection == RIGHT ? "RIGHT" :
                    currentDirection == BACKWARD ? "BACKWARD" :
                    "UNKNOWN"
                );
                xSemaphoreGive(xSerialSemaphore);
            }
        } else {
                Serial.println("IR Value");
            double distance = sensor.measureDistanceCm();
            int frontDistance = distance;

            if (frontDistance <= MAX_DISTANCE && frontDistance != -1) 
            {
                Utility::currentDirection = STOP;
                vTaskDelay(pdMS_TO_TICKS(500));
                Utility::currentDirection = BACKWARD;
                vTaskDelay(pdMS_TO_TICKS(500));
                Utility::currentDirection = STOP;
                vTaskDelay(pdMS_TO_TICKS(500));

                int maxDistance = 0;
                int maxAngle = 0;

                for (int angle = 0; angle <= 180; angle += SCAN_STEP) {
                    Utility::positionInDegree(angle);
                    vTaskDelay(pdMS_TO_TICKS(1));

                    distance = sensor.measureDistanceCm();

                    int index = angle / SCAN_STEP;
                    Utility::distanceAtAngle[index] = distance;

                    if (distance > maxDistance) {
                        maxDistance = distance;
                        maxAngle = angle;
                    }
                }

                Utility::currentDirection = getDirectionFromAngle(maxAngle);
                Utility::positionInDegree(90);
                vTaskDelay(pdMS_TO_TICKS(500));

                if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) 
                {
                    Serial.print("frontDistance: ");
                    Serial.print(frontDistance);
                    Serial.print(" | Max Distance: ");
                    Serial.print(maxDistance);
                    Serial.print(" cm at angle: ");
                    Serial.println(maxAngle);
                    Serial.print("Recommended Direction: ");
                    Serial.println(
                        currentDirection == LEFT ? "LEFT" :
                        currentDirection == RIGHT ? "RIGHT" :
                        currentDirection == FORWARD ? "FORWARD" :
                        "UNKNOWN"
                    );
                    xSemaphoreGive(xSerialSemaphore);
                }
            }
              else
              {
                  Utility::currentDirection = FORWARD;
              }

        }

      

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
