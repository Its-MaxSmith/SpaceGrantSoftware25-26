
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

class Motor {
private:
  int ena, in1, in2, enc1, enc2;
public:
  Encoder encoder;
  Motor(int ENApin, int IN1pin, int IN2pin, int encPin1, int encPin2)
    : ena(ENApin), in1(IN1pin), in2(IN2pin), enc1(encPin1), enc2(encPin2), encoder(encPin1, encPin2) {}

  void initializePins() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enc1, INPUT_PULLUP);
    pinMode(enc2, INPUT_PULLUP);
  }

  // Encoder functions
  int32_t readCounts() {
    return encoder.read();
  }

  void resetCounts() {
    encoder.write(0);
  }


  // Motor driver functions
  void run(float speed) {
    int pwm, reverse;
    if (speed >= 0) {
      reverse = 0;
      pwm = int(speed);
    } else {
      reverse = 1;
      pwm = abs(int(speed));
    }
    if (pwm > 255) {
      pwm = 255;
    }

    digitalWrite(in1, !reverse);
    digitalWrite(in2, reverse);
    analogWrite(ena, pwm);
  }

  void stop() {
    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
    analogWrite(ena, 0);
  }
};