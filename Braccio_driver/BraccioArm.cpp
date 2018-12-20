#include "BraccioArm.hpp"

#define BRACCIO_BOARD_PIN 12

#define BASE_PIN 11
#define SHOULDER_PIN 10
#define ELBOW_PIN 9
#define WRIST_PIN 6
#define WRISTROTATE_PIN 5
#define GRIPPER_PIN 3


BraccioArm::BraccioArm()
{
  initConstraints();
}

void BraccioArm::init()
{
  digitalWrite(BRACCIO_BOARD_PIN, LOW);
  servos[braccio::BASE] = ServoControl(BASE_PIN);
  servos[braccio::SHOULDER] = ServoControl(SHOULDER_PIN);
  servos[braccio::ELBOW] = ServoControl(ELBOW_PIN);
  servos[braccio::WRIST] = ServoControl(WRIST_PIN);
  servos[braccio::WRISTROTATE] = ServoControl(WRISTROTATE_PIN);
  servos[braccio::GRIPPER] = ServoControl(GRIPPER_PIN);

  digitalWrite(BRACCIO_BOARD_PIN, HIGH);
}

bool BraccioArm::setTarget(braccio::Servos servo, uint16_t pwm, uint16_t ms)
{
  if (pwm < constraints[servo][0] || pwm > constraints[servo][1])
  {
    return false;
  }
  debug(F("Setting target: "));
  debug(servo);
  debug(F(" To: "));
  debug(pwm);
  debug(F(" In: "));
  debug(ms);
  return servos[servo].setTarget(pwm, ms);
}

void BraccioArm::update()
{
  for (size_t i = 0; i < DOFS; ++i)
  {
    servos[i].updatePos();
  }
}

/*    PRIVATE FUNCTIONS     */
void BraccioArm::initConstraints()
{
  constraints[braccio::BASE][0] = 550;
  constraints[braccio::BASE][1] = 2400;

  constraints[braccio::SHOULDER][0] = 1000;
  constraints[braccio::SHOULDER][1] = 2300;

  constraints[braccio::ELBOW][0] = 600;
  constraints[braccio::ELBOW][1] = 2400;

  constraints[braccio::WRIST][0] = 550;
  constraints[braccio::WRIST][1] = 2400;

  constraints[braccio::WRISTROTATE][0] = 500;
  constraints[braccio::WRISTROTATE][1] = 2350;

  constraints[braccio::GRIPPER][0] = 900;
  constraints[braccio::GRIPPER][1] = 1550;

}
