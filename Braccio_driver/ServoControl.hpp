#ifndef SERVOCONTROL_HPP_
#define SERVOCONTROL_HPP_

#include <Arduino.h>
#include <Servo.h>

#define STEP_TIME 10  // ms

class ServoControl
{
  public:
    ServoControl();
    ServoControl(uint8_t pin);
    bool setTarget(unsigned long pwm, unsigned long ms);
    void updatePos();

    unsigned long curPosition;

  private:
    bool initialised;
    Servo servo;
    unsigned long stepSize /*ms*/;
    unsigned long target;
    unsigned long previousUpdate;
};
#endif  // SERVOCONTROL_HPP_
