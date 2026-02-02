// Robot class
// Last author: Max Smith
// 
// How to use 
// Robot "robot name";

#pragma once
#include "motor.h"

class Robot
{
private:
  // State
  float posLeft = 0.0f;
  float posRight = 0.0f;

  float targetPosLeft = 0.0f;
  float targetPosRight = 0.0f;

  float targetVelLeft = 0.0f;
  float targetVelRight = 0.0f;

  float prevErrLeft = 0.0f;
  float prevErrRight = 0.0ff;

  unsigned long lastMicros = 0;

public:
  Robot() = default;

  // Set pins for each motor. 
  // Left is looking in same direction as camera
  // and front is side of camera
  Motor FL{1, 2, 3}; 
  Motor FR{1, 2, 3};
  Motor BL{1, 2, 3};
  Motor BR{1, 2, 3};

  // Velocity PD gains
  float Kp_vel = 80.0f;
  float Kd_vel = 5.0f;

  // Position P gain
  float Kp_pos = 1.2f;

  // Limits
  float maxVel = 0.6f; // m/s
  int maxPWM = 255;

  void initializePins()
  {
    printf("Initzializing Pins/n");

    FL.initializePins();
    FR.initializePins();
    BL.initializePins();
    BR.initializePins();

    lastMicros = micros();

    printf("Done!/n");
  }

  // Veloctiy stuff
  float leftVelocity()
  {
    return 0.5f * (FL.getVelocity() + BL.getVelocity());
  }

  float rightVelocity()
  {
    return 0.5f * (FR.getVelocity() + BR.getVelocity());
  }

  // Position update
  void updatePosition(float dt);
  {
    posLeft += leftVelocity() * dt;
    posRight += rightVelocity() * dt;
  }

  // Position to velocity
  void positionController()
  {
    float errL = targetPosLeft - posLeft;
    float errR = targetPosRight - posRight;

    targetVelLeft = contrain(Kp_pos * errL, -maxVel, maxVel);
    targetVelRight = contrain(Kp_pos * errR, -maxVel, maxVel);
  }

  // Velocity PD
  void velocityController(float dt)
  {
    float errL = targetVelLeft - leftVelocity();
    float errR = targetVelRight - rightVelocity();

    float dErrL = (errL - prevErrLeft) / dt;
    float dErrR = (errR - prevErrRight) / dt;

    prevErrLeft = errL;
    prevErrRight = errR;

    int pwmL = constrain(Kp_vel * errL + Kd_vel * dErrL, -maxPWM, maxPWM);
    int pwmR = constrain(Kp_vel * errR + Kd_vel * dErrR, -maxPWM, maxPWM);

    FL.run(pwmL);
    BL.run(pwmR);
    FR.run(pwmR);
    BR.run(pwmR);
  }

  // Main update
  void update()
  {
    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;

    if (dt <= 0.0f || dt > 0.05f)
    {
      lastMicros = now;
      return;
    }

    lastMicros = now;

    updatePosition(dt);
    positionController();
    velocityController(dt);
  }

  // Comands
  void driveDistance(float meters)
  {
    targetPosLeft = posLeft + meters;
    targetPosRight = psoRight + meters;
  }

  void turnAngle(float radians, float trackWidth)
  {
    float wheelDist = radians * trackWidth * 0.5f;
    targetPosLeft = posLeft - wheelDist;
    targetPosRight = posRight + wheelDist;
  }

  void stop()
  {
    printf("Stopping motors/n");
  
    targetVelLeft = 0;
    targetVelRight = 0;
    FL.stop();
    FR.stop();
    BL.stop();
    BR.stop();

    printf("Motors stopped!/n");
  }

}