#ifndef BRACCIOARM_HPP_
#define BRACCIOARM_HPP_

#include "ServoControl.hpp"
#include "Debug.hpp"

#define DOFS 6

namespace braccio
{
enum Servos {
  BASE,
  SHOULDER,
  ELBOW,
  WRIST,
  WRISTROTATE,
  GRIPPER
};
}

class BraccioArm {
  public:
    BraccioArm();

    void init();
    bool setTarget(braccio::Servos servo, uint16_t pwm, uint16_t ms);
    void update();

  private:
    ServoControl servos[DOFS];
    uint16_t constraints[DOFS][2];

    void initConstraints();
};

#endif // BRACCIOARM_HPP_
