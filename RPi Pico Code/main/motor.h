// Motor class
// Last author: Max Smith
// 
// How to use
// Motor "motor name"{IN1 pin, IN2 pin, ENA pin}

class Motor 
{
private: 
  int ena, in1, in2;
private:
  Motor(int IN1pin, int IN2pin, int ENApin)
    : in1(IN1pin), in2(IN2pin), ena(ENApin)

  // Set pins as output
  void initializePins()
  {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
  }

  // Drive forward

  // Turn
};
