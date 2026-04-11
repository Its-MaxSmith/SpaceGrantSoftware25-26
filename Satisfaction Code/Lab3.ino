
int poses[3][8] = {
  { 140, 140, 200, 200, 100, 100, 0, 0 },
  { 0, 90, 90, -90, -90, -150, -150, 0 },
  { 90, 0, -90, -180, -90, -180, -270, 0 }
};

#include "robot.h"
Robot myRobot;

int pushButton = 50;
int ran_course = 0;

/////////////////////////////////////////////////////////////////

// gains
float Kp_v = 5;  
float Ki_v = 0.02;
float Kd_v = 0.1; 
float Kp_pos_v = 1;
float Kv_v = 0.5; 
float Ks_v = 2;

float Kp_w = 8;
float Ki_w = 0.01;
float Kd_w = 0.05;
float Kp_pos_w = 1;
float Kv_w = 1;
float Ks_w = 1;

float V_MAX = 83.17 * 0.95; // cm/s
float W_MAX = 663.27 * 0.95; // deg/s

float wheelBase = 25; // ?? it works cm

/////////////////////////////////////////////////////////////////


void setup() 
{
  Serial.begin(9600);

  myRobot.initializePins();

  myRobot.set_wheelbase(wheelBase);

  myRobot.L_speedControl.setGains(Kp_v, Ki_v, Kd_v, V_MAX, Kp_pos_v, Kv_v, Ks_v);
  myRobot.R_speedControl.setGains(Kp_v, Ki_v, Kd_v, V_MAX, Kp_pos_v, Kv_v, Ks_v);
  myRobot.headingControl.setGains(Kp_w, Ki_w, Kd_w, W_MAX, Kp_pos_w, Kv_w, Ks_w);


  Serial.println("Setup done");
}

void loop() 
{
  /*
  if (!ran_course) {
    // Holds car still for measurement until push button is pressed
    delay(1000);
    for (int i = 0; i < 8; i++) 
    {
      // Drive to target pose
      myRobot.goToPose(poses[0][i], poses[1][i], poses[2][i]);
      // Print current pose: x, y, theta
      Serial.print(myRobot.pose.x);
      Serial.print(", ");
      Serial.print(myRobot.pose.y);
      Serial.print(", ");
      Serial.println(myRobot.pose.theta);
    }
    ran_course = 1;
  } */

  myRobot.testAngularLimits();
  myRobot.testLinearLimits();

}
