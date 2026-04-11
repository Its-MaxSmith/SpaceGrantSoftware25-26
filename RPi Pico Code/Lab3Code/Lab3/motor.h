#define ENCODER_USE_INTERRUPTS
#include <PicoEncoder.h>

class Motor 
{
  private:
    int ena, in1, in2;
    int encA, encB;

    volatile int32_t count = 0;

    // ISR routing
    static Motor* instances[4];

    static void isr0() { if (instances[0]) instances[0]->handleISR(); }
    static void isr1() { if (instances[1]) instances[1]->handleISR(); }
    static void isr2() { if (instances[2]) instances[2]->handleISR(); }
    static void isr3() { if (instances[3]) instances[3]->handleISR(); }

    void (*isrFunc)();

  public:
    Motor(int ENA, int IN1, int IN2, int A, int B, int index)
      : ena(ENA), in1(IN1), in2(IN2), encA(A), encB(B)
    {
      if (index >= 0 && index < 4)
        instances[index] = this;

      switch (index) {
        case 0: isrFunc = isr0; break;
        case 1: isrFunc = isr1; break;
        case 2: isrFunc = isr2; break;
        case 3: isrFunc = isr3; break;
        default: isrFunc = isr0; break;
      }
    }

    void begin() {
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(ena, OUTPUT);

      pinMode(encA, INPUT_PULLUP);
      pinMode(encB, INPUT_PULLUP);

      // Attach BOTH channels
      attachInterrupt(encA, isrFunc, CHANGE);
      attachInterrupt(encB, isrFunc, CHANGE);
    }

    // Fast quadrature decoding
    void handleISR() {
      count++;
      /*bool A = digitalRead(encA);
      bool B = digitalRead(encB);

      // XOR gives direction
      if (A ^ B)
        count++;
      else
        count--;*/
    }

    // Safe read
    int32_t read() {
      noInterrupts();
      int32_t c = count;
      interrupts();
      return c;
    }

    void reset() {
      noInterrupts();
      count = 0;
      interrupts();
    }

    void run(float speed) {
      int pwm = constrain(abs((int)speed), 0, 255);
      bool reverse = speed < 0;

      digitalWrite(in1, !reverse);
      digitalWrite(in2, reverse);
      analogWrite(ena, pwm);
    }

    void stop() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(ena, 0);
    }
};

// Static array definition
Motor* Motor::instances[4] = { nullptr, nullptr, nullptr, nullptr };
