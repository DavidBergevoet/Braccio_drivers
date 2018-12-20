#include "ServoControl.hpp"

#define BEGIN_POS 1500
#define MIN_PWM 500
#define MAX_PWM 2500

ServoControl::ServoControl(): curPosition(BEGIN_POS), initialised(false), stepSize(10),
  target(BEGIN_POS), previousUpdate(0)
{
}

ServoControl::ServoControl(uint8_t pin) : curPosition(BEGIN_POS), initialised(true),
  stepSize(10), target(BEGIN_POS), previousUpdate(0)
{
  servo.attach(pin);
}

bool ServoControl::setTarget(unsigned long pwm, unsigned long ms)
{
  if (!initialised || pwm < MIN_PWM || pwm > MAX_PWM) {
    return false;
  }

  long sS = (abs((int)pwm - (int)curPosition) / (ms / STEP_TIME));
  if (sS == 0)
  {
    return false;
  }
  target = pwm;
  stepSize = (uint32_t)sS;
  return true;
}

void ServoControl::updatePos()
{
  if (millis() - previousUpdate >= STEP_TIME) {
    previousUpdate = millis();
    if (abs(target - curPosition) < stepSize)
    {
      curPosition = target;
    }
    else if (target < curPosition)
    {
      curPosition -= stepSize;
    }
    else if (target > curPosition)
    {
      curPosition += stepSize;
    }
    servo.writeMicroseconds(curPosition);
  }
}
