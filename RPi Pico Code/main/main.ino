
#include "robot.h"
Robot myRobot;

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

float wheelBase = 29; // ?? it works cm

/////////////////////////////////////////////////////////////////


void setup() 
{
  Serial.begin(9600);

  myRobot.initializePins();

  myRobot.set_wheelbase(wheelBase);

  pinMode(pushButton, INPUT_PULLUP);

  myRobot.L_speedControl.setGains(Kp_v, Ki_v, Kd_v, V_MAX, Kp_pos_v, Kv_v, Ks_v);
  myRobot.R_speedControl.setGains(Kp_v, Ki_v, Kd_v, V_MAX, Kp_pos_v, Kv_v, Ks_v);
  myRobot.headingControl.setGains(Kp_w, Ki_w, Kd_w, W_MAX, Kp_pos_w, Kv_w, Ks_w);

}

void loop() 
{
  // Read from camera
  // Dertermine the target theta
  // Determine x and y target from distance and new theta
  // Call goToPose
    
}
