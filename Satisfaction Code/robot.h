// Lab 3 Robot class with Twist class
//
// Author: Max Smith
// Date: 3/4/2026
//

#include "motor.h"
#include "pose.h"
#include "controller.h"

// Creates a Twist class to hold
// linear, angular velocity
class Twist {
public:
  float linear = 0.0;
  float angular = 0.0;
  Twist() = default;
  void set(float vel, float ang_vel) {
    linear = vel;
    angular = ang_vel;
  }
};



// State machine
enum NavState
{
  DRIVE_TO_POINT,
  TURN_TO_ANGLE,
  FINISHED
};



class Robot {
public:

  Controller L_speedControl;  
  Controller R_speedControl;
  Controller headingControl;

  Pose pose;

  Twist cmd_vel;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  float WHEEL_RADIUS = 6.25; // cm
  float COUNTS_PER_REV = 1600; 

  float XY_TOLERANCE = 2.0; // cm
  float ANGLE_TOLERANCE = 0.5; // deg

  int MAX_PWM = 255;
  int MIN_PWM = 100; // Was 150

  // Motion limits
  float V_MAX = 83.17 * 0.95; // cm/s
  float V_ACC = 1094.32 * 0.95; // cm/s^2
  float W_MAX = 663.27 * 0.95; // deg/s
  float W_ACC = 3316.36 * 0.95; // deg/s^2

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  // Global Variables
  float lastTime = 0;

  float v_profile = 0.0;
  float w_profile = 0.0;

  long lastFL = 0;
  long lastBL = 0;
  long lastFR = 0;
  long lastBR = 0;

  long lastL = 0;
  long lastR = 0;

  float vL_filt = 0;
  float vR_filt = 0;

  const unsigned long CONTROL_PERIOD = 10; // 100 Hz

  Robot() = default;

  // Motors (UPDATED for RP2040 interrupt system)
  Motor FL{ 4, 2, 3, 1, 0, 0 };    // Front Left
  Motor BL{ 14, 12, 13, 19, 20, 1 }; // Back Left 11 10
  Motor FR{ 9, 7, 8, 6, 5, 2 };   // Front Right
  Motor BR{ 16, 18, 17, 11, 10, 3 }; // Back Right  19 20

  void initializePins() {
    FL.begin();
    BL.begin();
    FR.begin();
    BR.begin();
  }

  // Set the wheelbase of the robot
  void set_wheelbase(float width) 
  {
    pose.wheelbase = width;
  }

  // Resets the encoder counts held within the motor class
  void resetCounts() 
  {
    FL.reset();
    BL.reset();
    FR.reset();
    BR.reset();
  }

  // Sets the pwm for each motor
  void runAll(int FLpwm, int BLpwm, int FRpwm, int BRpwm) 
  {
    FL.run(FLpwm);
    BL.run(BLpwm);
    FR.run(FRpwm);
    BR.run(BRpwm);
  }

  // Sets all pwms to 0
  void stopAll()
  {
    FL.stop();
    BL.stop();
    FR.stop();
    BR.stop();
  }

  // Main driving function that moves the robot to the given target position
  void goToPose(float x, float y, float theta)
  {
    // Resets
    L_speedControl.reset();
    R_speedControl.reset();
    headingControl.reset();
    v_profile = 0.0;
    w_profile = 0.0;
    resetCounts();
    pose.set(pose.x, pose.y, pose.theta);
    lastFL = lastBL = lastFR = lastBR = 0;
    lastL = lastR = 0;

    Pose target;
    target.set(x, y, theta);

    char navState = TURN_TO_ANGLE;

    lastTime = millis();

    while (navState != FINISHED)
    {
      unsigned long now = millis();

      if (now - lastTime < CONTROL_PERIOD)
        continue;

      float dt = CONTROL_PERIOD / 1000.0;
      lastTime = now;

      /* ----------------- Update Odometry ------------------ */

      updateOdometry(dt);

      /* ---------------- State Machine --------------------- */

      switch (navState)
      {
        // Turns the robot to the desired angle
        case TURN_TO_ANGLE:
        {
          float err = target.theta - pose.theta;

          err = atan2(
            sin(err * PI / 180),
            cos(err * PI / 180)
          ) * 180 / PI;

          if (fabs(err) < ANGLE_TOLERANCE)
          {
            navState = DRIVE_TO_POINT;
            cmd_vel.set(0, 0);
            break;
          }

          float w =
            headingControl.getCmd(err, 0, dt);

          w = clamp(w, -W_MAX, W_MAX);

          cmd_vel.set(0, w);

          break;
        }
        // Moves robot to the next x y position
        case DRIVE_TO_POINT:

          if (atXY(target))
          {
            navState = FINISHED;

            headingControl.reset();

            cmd_vel.set(0, 0);

            break;
          }

          computeCmd(target, dt);

          break;


        // Sets velocities to 0 and stops robot
        case FINISHED:
          cmd_vel.set(0, 0);
          break;
      }

      // Velocity Profiling 
      updateVelocityProfile(dt);

      //Wheel References
      float vL_ref, vR_ref;
      twistToWheels(vL_ref, vR_ref);

      // Measure Velocity 
      float vL_meas, vR_meas;
      getWheelVel(dt, vL_meas, vR_meas);

      //Control
      int pwmL, pwmR;

      computePWM(vL_meas, vR_meas, vL_ref, vR_ref, pwmL, pwmR, dt);

      runAll(pwmL, pwmL, pwmR, pwmR);

      // Debug

      /*static unsigned long lastPrint = 0;

      if (millis() - lastPrint > 200)
      {
        lastPrint = millis();

        Serial.print("x=");
        Serial.print(pose.x);
        Serial.print(" y=");
        Serial.print(pose.y);
        Serial.print(" t=");
        Serial.println(pose.theta);
      }*/
    }

    stopAll();
  }


  // Uses the counts and time to find the distance that the robot has traveled from last call and updates 
  // the the pose class to refelct the traveled distance
  void updateOdometry(float dt)
  {
    long FL_c = FL.read();
    long BL_c = BL.read();
    long FR_c = FR.read();
    long BR_c = BR.read();

    long dFL = FL_c - lastFL;
    long dBL = BL_c - lastBL;
    long dFR = FR_c - lastFR;
    long dBR = BR_c - lastBR;

    lastFL = FL_c;
    lastBL = BL_c;
    lastFR = FR_c;
    lastBR = BR_c;

    float wheelCirc = 2.0 * PI * WHEEL_RADIUS;

    float dL = ((dFL + dBL) / 2) / COUNTS_PER_REV * wheelCirc;
    float dR = ((dFR + dBR) / 2) / COUNTS_PER_REV * wheelCirc;

    float dC = (dL + dR) / 2;
    float dTheta = (dR - dL) / pose.wheelbase;
    float dTheta_deg = dTheta * 180 / PI;

    float thetaMid = pose.theta + 0.5 * dTheta_deg;

    pose.x += dC * cos(thetaMid * PI / 180.0);
    pose.y += dC * sin(thetaMid * PI / 180.0);

    pose.theta += dTheta_deg;

    return;
  }


  // Calculates the velocity of the left and right wheels by using the counts per revolution and the time from last call
  void getWheelVel(float dt, float &vL, float &vR)
  {
    long L = (FL.read() + BL.read()) / 2;
    long R = (FR.read() + BR.read()) / 2;

    long dL = L - lastL;
    long dR = R - lastR;

    lastL = L;
    lastR = R;

    float wheelCirc = 2 * PI * WHEEL_RADIUS;

    float distL = dL / COUNTS_PER_REV * wheelCirc;
    float distR = dR / COUNTS_PER_REV * wheelCirc;

    vL = distL / dt;
    vR = distR / dt;

    // Low-pass filter
    const float alpha = 0.7;

    vL_filt = alpha * vL_filt + (1 - alpha) * vL;
    vR_filt = alpha * vR_filt + (1 - alpha) * vR;

    vL = vL_filt;
    vR = vR_filt;
  }


  // Calculates the wanted pwm for left and right by using the controller class
  void computePWM(float vL_meas, float vR_meas, float vL_ref, float vR_ref, int &pwmL, int &pwmR, float dt)
  {
    float eL = vL_ref - vL_meas;
    float eR = vR_ref - vR_meas;

    float uL = L_speedControl.getCmd(eL, vL_ref, dt);
    float uR = R_speedControl.getCmd(eR, vR_ref, dt);

    pwmL = applyMinPWM(uL);
    pwmR = applyMinPWM(uR);
  }


  // Applys a minimum pwm so that the  motors don't stall
  int applyMinPWM(float u)
  {
      float deadzone = 5;

      if (fabs(u) < deadzone)
          return 0;

      float sign = (u > 0) ? 1.0 : -1.0;

      float pwm = fabs(u);

      // Smooth offset instead of hard jump
      pwm = MIN_PWM + (pwm * (MAX_PWM - MIN_PWM) / MAX_PWM);

      return constrain(sign * pwm, -MAX_PWM, MAX_PWM);
  }


  // Calculates a position target from the target and the time and uses a controller to ensure that it stays on target
  void computeCmd(Pose target, float dt)
  {
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;

    float targetAngle = atan2(dy, dx) * 180 / PI;

    float headingErr = targetAngle - pose.theta;

    headingErr = atan2(sin(headingErr * PI / 180), cos(headingErr * PI / 180)) * 180 / PI;

    float distErr = sqrt(dx*dx + dy*dy);

    float v_raw = L_speedControl.getCmd_pos(distErr);

    float headingScale = cos(headingErr * PI / 180.0);
    headingScale = max(0.0, headingScale); 

    v_raw *= headingScale;

    float w_raw = headingControl.getCmd(headingErr, 0.0, dt);
    if (fabs(headingErr) > 45)
      {
        v_raw = 0;   // don't drive forward if facing wrong direction
      }

    // clamp speeds
    v_raw = clamp(v_raw, -V_MAX, V_MAX);
    w_raw = clamp(w_raw, -W_MAX, W_MAX);

    cmd_vel.set(v_raw, w_raw);

    return;
  }


  // Finds the velocities needed for rotating using the wheelbase
  void twistToWheels(float &vL, float &vR)
  {
    float v = v_profile;
    float w = w_profile * PI / 180;

    vL = v - (w * pose.wheelbase / 2);
    vR = v + (w * pose.wheelbase / 2);

    return;
  }


  // Checks to see if the robot has reached the desired distance
  bool atXY(Pose target)
  {
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;

    float dist = sqrt(dx*dx + dy*dy);

    return(dist < XY_TOLERANCE);
  }


  // Checks to see if the robot has reached the desired angle
  bool atTheta(Pose target)
  {
    float err = target.theta - pose.theta;

    err = atan2(sin(err * PI / 180), cos(err * PI / 180)) * 180 / PI;

    return(fabs(err) < ANGLE_TOLERANCE);
  }


  // Stepps up or down the trapezoid velocity profile based on difference from the target
  float trapezoidStep(float current, float target, float accel, float dt)
  {
    float diff = target - current;
    float step = accel * dt;

    if(fabs(diff) <= step) return(target);

    if(diff > 0) return(current + step);
    else return(current - step);
  }


  // Clamps a variable between two set values
  float clamp(float val, float minVal, float maxVal)
  {
    if(val < minVal) return(minVal);
    if(val > maxVal) return(maxVal);
    return(val);
  }


  // Updates the linear and angular velocity using the trapazoid step
  void updateVelocityProfile(float dt)
  {
    // Linear
    v_profile = trapezoidStep(v_profile, cmd_vel.linear, V_ACC, dt);

    // Angular
    w_profile = trapezoidStep(w_profile, cmd_vel.angular, W_ACC, dt);
  }



  // Motion Limit tests
  void testLinearLimits(float testTime = 2.0)
  {
    Serial.println("Testing Linear Limits");

    resetCounts();
    stopAll();
    delay(500);

    unsigned long startTime = millis();
    unsigned long prevTime  = millis();

    float prevVel = 0.0;
    float maxVel  = 0.0;
    float maxAcc  = 0.0;

    while ((millis() - startTime) < testTime * 1000)
    {
      runAll(MAX_PWM, MAX_PWM, MAX_PWM, MAX_PWM);

      unsigned long now = millis();
      float dt = (now - prevTime) / 1000.0;

      // Ignore tiny dt 
      if (dt < 0.02) continue;

      prevTime = now;

      float vL, vR;
      getWheelVel(dt, vL, vR);

      float vel = (vL + vR) / 2.0;
      float acc = (vel - prevVel) / dt;

      if (fabs(vel) > maxVel)
        maxVel = fabs(vel);

      if (fabs(acc) > maxAcc && fabs(acc) < 5000)  // sanity clamp
        maxAcc = fabs(acc);

      prevVel = vel;
    }

    stopAll();

    Serial.print("Max Linear Velocity (cm/s): ");
    Serial.println(maxVel);

    Serial.print("Max Linear Acceleration (cm/s^2): ");
    Serial.println(maxAcc);
  }


  void testAngularLimits(float testTime = 2.0)
  {
    Serial.println("Testing Angular Limits");

    resetCounts();
    stopAll();
    delay(500);

    unsigned long startTime = millis();
    unsigned long prevTime  = millis();

    float prevOmega = 0.0;
    float maxOmega  = 0.0;
    float maxAlpha  = 0.0;

    while ((millis() - startTime) < testTime * 1000)
    {
      runAll(-MAX_PWM, -MAX_PWM, MAX_PWM, MAX_PWM);

      unsigned long now = millis();
      float dt = (now - prevTime) / 1000.0;

      if (dt < 0.02) continue;   // 🔒 prevents dt explosion

      prevTime = now;

      float vL, vR;
      getWheelVel(dt, vL, vR);

      float omega_rad = (vR - vL) / pose.wheelbase;
      float omega_deg = omega_rad * 180.0 / PI;

      float alpha = (omega_deg - prevOmega) / dt;

      if (fabs(omega_deg) > maxOmega)
        maxOmega = fabs(omega_deg);

      if (fabs(alpha) > maxAlpha && fabs(alpha) < 10000)  // sanity clamp
        maxAlpha = fabs(alpha);

      prevOmega = omega_deg;
    }

    stopAll();

    Serial.print("Max Angular Velocity (deg/s): ");
    Serial.println(maxOmega);

    Serial.print("Max Angular Acceleration (deg/s^2): ");
    Serial.println(maxAlpha);
  }

  void testAllMotorsForward() 
  {
    Serial.println("Running all motors forward at full PWM");

    // Run all motors at max forward
    FL.run(255);
    BL.run(255);
    FR.run(255);
    BR.run(255);
  }



  void testEncodersContinuous()
  {
    Serial.println("=== Continuous Encoder Test ===");

    resetCounts();
    delay(500);

    unsigned long lastPrint = 0;

    while (true)
    {
      // Read raw counts
      long FL_c = FL.read();
      long BL_c = BL.read();
      long FR_c = FR.read();
      long BR_c = BR.read();

      // Compute left/right averages
      long L = (FL_c + BL_c) / 2;
      long R = (FR_c + BR_c) / 2;

      // Print at ~10 Hz
      if (millis() - lastPrint > 100)
      {
        lastPrint = millis();

        Serial.print("FL: "); Serial.print(FL_c);
        Serial.print(" | BL: "); Serial.print(BL_c);
        Serial.print(" | FR: "); Serial.print(FR_c);
        Serial.print(" | BR: "); Serial.print(BR_c);

        Serial.print(" || L avg: "); Serial.print(L);
        Serial.print(" | R avg: "); Serial.println(R);
      }
    }
  }

};


