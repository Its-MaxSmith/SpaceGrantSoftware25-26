// PID Contoller

class Controller {
public:
  float Kp, Ki, Kd, maxSpeed;
  float Kp_pos;
  float Kv;
  float Ks;

  float prevError = 0;
  float integral = 0;
  float dFilt = 0;

  Controller()
    : Kp(0.0), Ki(0.0), Kd(0.0), maxSpeed(0.0), Kp_pos(0.0), Kv(0.0), Ks(0.0) {}

  Controller(float kp, float ki, float kd, float maxspeed, float kp_pos)
    : Kp(kp), Ki(ki), Kd(kd), maxSpeed(maxspeed), Kp_pos(kp_pos) {}

  void setGains(float kp, float ki, float kd, float maxspeed, float kp_pos, float kv, float ks) 
  {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    maxSpeed = maxspeed;
    Kp_pos = kp_pos;
    Kv = kv;
    Ks = ks;
  };

  void reset()
  {
    prevError = 0.0;
    integral = 0.0;
    dFilt = 0;
  }



  float getCmd_pos(float error)
  {
    float u = Kp_pos * error;

    u = constrain(u, -maxSpeed, maxSpeed);

    return u;
  }



  float feedforward(float targetVel)
  {
    if(fabs(targetVel) < 0.01) return(0.0);

    float sign = (targetVel > 0) ? 1.0 : -1.0;

    return Kv * targetVel + Ks * sign;
  }



  float getCmd(float error, float targetVel, float dt) 
  {

    integral += error * dt;

    // Anti windup
    float maxInt = maxSpeed / max(0.001, Ki);
    integral = constrain(integral, -maxInt, maxInt);

    float rawD = (error - prevError) / dt;

    const float dAlpha = 0.8;
    dFilt = dAlpha * dFilt + (1 - dAlpha) * rawD;

    prevError = error;

    float pid = (Kp * error) + (Ki * integral) + (Kd * dFilt);

    float ff = feedforward(targetVel);

    float u = pid + ff;

    u = constrain(u, -maxSpeed, maxSpeed);

    return u;
  }
};