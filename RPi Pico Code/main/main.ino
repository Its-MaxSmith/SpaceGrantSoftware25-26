// Main program
// Last author: Max Smith
//
// Code for the pi pico to get information from the main pi and
// controls the motors
//
// This main needs to be changed for each robot

// Needed header files
#include "robot.h"

Robot NOMAD;

void setup() 
{
  NOMAD.initializePins();

  // Set the speeds for motor to turn acuratly
  NOMAD.driveSpeed_forward = 1.0; // Needs to be changed per robot
  NOMAD.driveSpeed_reverse = 1.0; // Needs to be changed per robot
  NOMAD.turnSpeed_left = 1.0; // Needs to be changed per robot
  NOMAD.turnSpeed_right = 1.0; // Needs to be changed per robot
}

void loop() 
{
  // wait for directions from pi

  // act on direction

}
