
class Pose {
public:
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
  // wheelbase = width from wheel to wheel in cm
  float wheelbase = 10.0;

  Pose() = default;

  // NOTE: Changed "update" to "set"
  void set(float x_in, float y_in, float theta_in) {
    x = x_in;
    y = y_in;
    theta = theta_in;
  }

  // dL, dR are left and right encoder increments
  // NOTE: dL, dR expected in cms
  void update(float dL, float dR) {
    float ds = 0.5 * (dL + dR);
    float dTheta = (dR - dL) / wheelbase * 180/PI;
    x += ds * cos((theta + 0.5 * dTheta)*PI/180);
    y += ds * sin((theta + 0.5 * dTheta)*PI/180);
    theta += dTheta;
  }
};
