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
    bool setTarget(uint16_t pwm, uint16_t ms);
    void updatePos();

    uint16_t getCurPosition()const;
    

  private:
  uint16_t curPosition;
    bool initialised;
    Servo servo;
    uint16_t stepSize /*ms*/;
    uint16_t  target;
    uint16_t previousUpdate;
};
#endif  // SERVOCONTROL_HPP_
