#include "ServoControl.hpp"
#include "Debug.hpp"

#define BEGIN_POS 1500
#define MIN_PWM 500
#define MAX_PWM 2500

#define MAX_STEPS 10

ServoControl::ServoControl()
  : curPosition(BEGIN_POS), initialised(false), stepSize(10), target(BEGIN_POS), previousUpdate(0)
{
}

ServoControl::ServoControl(uint8_t pin)
  : curPosition(BEGIN_POS), initialised(true), stepSize(10), target(BEGIN_POS), previousUpdate(0)
{
  servo.attach(pin);
}

bool ServoControl::setTarget(uint16_t pwm, uint16_t ms)
{
  if (!initialised || pwm < MIN_PWM || pwm > MAX_PWM)
  {
    return false;
  }

  int sS = (abs((int)pwm - (int)curPosition) / (ms / STEP_TIME));
  if (sS == 0)
  {
    sS = 1;
  }
  if (sS >= MAX_STEPS)
  {
    sS = MAX_STEPS;
  }
  target = pwm;
  stepSize = (uint32_t)sS;
  debug(F("Size: "));
  debugln(stepSize);
  return true;
}

void ServoControl::updatePos()
{
  if (millis() - previousUpdate >= STEP_TIME)
  {
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

uint16_t ServoControl::getCurPosition() const
{
  return curPosition;
}
