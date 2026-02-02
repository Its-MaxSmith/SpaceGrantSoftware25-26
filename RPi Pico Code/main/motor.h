// Motor class
// Last author: Max Smith
// 
// How to use
// Motor "motor name"{IN1 pin, IN2 pin, ENA pin}

#pragma once
#include <Arduino.h>

class Motor 
{
private: 
  int ena, in1, in2;
  volatile float velocity = 0.0f;

public:
  Motor(int IN1pin, int IN2pin, int ENApin)
    : in1(IN1pin), in2(IN2pin), ena(ENApin) {}

  // Set pins as output
  void initializePins()
  {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    stop();
  }

  // Motor driver functions
  void run(int pwm)
  {
    pwm = contrain(pwm, -255, 255);

    if (speed >= 0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, pwm);
    }
    else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, -pwm);
    }
  }

  void stop()
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
  }

  float getVelocity()
  {
    // use encoders to get a velocity out
  }

  void setVelocity(float v)
  {
    velocity = v;
  }
  
};
