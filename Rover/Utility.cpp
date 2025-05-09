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
  Serial.println(positionInDegrees);
    myservo.write(positionInDegrees);
    // currentServoAngle = myservo.read();
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
         if (irValue == LOW && false)  // Obstacle detected via IR
        {
            

            // Utility::currentDirection = BACKWARD;
            // // motor.moveStop();
            // vTaskDelay(pdMS_TO_TICKS(300));

            // Utility::currentDirection = STOP;
            // vTaskDelay(pdMS_TO_TICKS(300));

            // int maxDistance = 0;
            // int maxAngle = 0;

            // for (int angle = 1; angle <= 180; angle += SCAN_STEP) {
            //     Utility::positionInDegree(angle);
            //     vTaskDelay(pdMS_TO_TICKS(1));

            //     double distance = sensor.measureDistanceCm();
            //     int index = angle / SCAN_STEP;

            //     Utility::distanceAtAngle[index] = distance;

            //     Direction dir = getDirectionFromAngle(angle);
            //     if (distance > maxDistance && dir != FORWARD) {
            //         maxDistance = distance;
            //         maxAngle = angle;
            //     }
            // }

            // Utility::currentDirection = getDirectionFromAngle(maxAngle);
            // Utility::positionInDegree(90);
           
            //     if (maxAngle > 90) {
            //       vTaskDelay(pdMS_TO_TICKS(500*((maxAngle-90)/90)));
            //     } else if (maxAngle < 90) {
            //       vTaskDelay(pdMS_TO_TICKS(500*((90-maxAngle)/90)));
            //     }

            // if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
            //     Serial.print("IR Triggered | Max Distance: ");
            //     Serial.print(maxDistance);
            //     Serial.print(" cm at angle: ");
            //     Serial.println(maxAngle);
            //     Serial.print("Chosen Direction (excluding FORWARD): ");
            //     Serial.println(
            //         currentDirection == LEFT ? "LEFT" :
            //         currentDirection == RIGHT ? "RIGHT" :
            //         currentDirection == BACKWARD ? "BACKWARD" :
            //         "UNKNOWN"
            //     );
            //     xSemaphoreGive(xSerialSemaphore);
            // }
        } else {
                
            double distance = sensor.measureDistanceCm();
            int frontDistance = distance;

            if (frontDistance <= MAX_DISTANCE && frontDistance != -1) 
            {
                Utility::currentDirection = STOP;
                vTaskDelay(pdMS_TO_TICKS(300));
                Utility::currentDirection = BACKWARD;
                vTaskDelay(pdMS_TO_TICKS(300));
                Utility::currentDirection = STOP;
                vTaskDelay(pdMS_TO_TICKS(300));

                int maxDistance = 0;
                int maxAngle = 0;

                for (int angle = 1; angle <= 180; angle += SCAN_STEP) {
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
                if (maxAngle > 90) {
                  Serial.println("Angle: ");
                Serial.println(maxAngle);
                Serial.println("MAx Distance: ");
                Serial.println(maxDistance);
                Serial.println("Delay: ");
                Serial.println(500*((float(maxAngle)-90)/90));
                  vTaskDelay(pdMS_TO_TICKS(500*((float(maxAngle)-90)/90)));
                } else if (maxAngle < 90) {
                  Serial.println("Angle: ");
                Serial.println(maxAngle);
                Serial.println("MAx Distance: ");
                Serial.println(maxDistance);
                Serial.println("Delay: ");
                Serial.println(500*((90-float(maxAngle))/90));
                  vTaskDelay(pdMS_TO_TICKS(500*((90-float(maxAngle))/90)));
                }


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
