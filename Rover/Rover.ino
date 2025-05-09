#include "Utility.hpp"
#include "DCMotor.hpp"




void setup() {
  Serial.begin(9600);

  Utility::createTask();  // Assuming this is properly defined elsewhere
  motor.setupMotors();
  
}

void loop() {
   

  switch (Utility::currentDirection) 
  {

    case LEFT:
      {motor.turnLeft();
      break;}

    case RIGHT:
      {motor.turnRight();
      break;}
    
    case BACKWARD:
      {motor.moveBackward();
      break;}

    case STOP:
      {
      motor.moveStop();
      
      break; } 

    case FORWARD:
      {motor.moveForward();
      break;}

    default:
      {motor.moveStop();
      break;}
  }

  
}
