// Robot class
// Last author: Max Smith
// 
// How to use 
// Robot "robot name";

#include "motor.h"

class Robot
{
public() = default;

  // Set pins for each motor. 
  // Left is looking in same direction as camera
  // and front is side of camera
  Motor FL{1, 2, 3}; 
  Motor FR{1, 2, 3};
  Motor BL{1, 2, 3};
  Motor BR{1, 2, 3};

  // Move speed per direction
  float driveSpeed_forward = 1.0; // m/s
  float driveSpeed_reverse = 1.0; // m/s
  float turnSpeed_left = 1.0; // rad/s
  float turnSpeed_right = 1.0; // rad/s

  // Initialize pins for motors
  void initializePins()
  {
    printf("Initializing pins\n");
    FL.initializePins();
    FR.initializePins();
    BL.initializePins();
    BR.initializePins();
    printf("Done\n");
  }

  // set angles between 180 and -180
  float wrapAngle(float angle)
  {
    while (angle > 180) angle -= 360;
    while (angle <= -180) angle += 360;
    return angle;
  }


};