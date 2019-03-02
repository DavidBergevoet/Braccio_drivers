#ifndef BRACCIOARM_HPP_
#define BRACCIOARM_HPP_

#include "ServoControl.hpp"
#include "Debug.hpp"

#define DOFS 6

/**
 * @brief The namespace used for the Braccio arm
 */
namespace braccio
{
/**
 * @brief An enum which is used for all the DOF's of the Braccio arm
 */
enum Servos
{
  BASE,
  SHOULDER,
  ELBOW,
  WRIST,
  WRISTROTATE,
  GRIPPER
};
}  // namespace braccio

/**
 * @brief A class which is used for controlling the Braccio arm
 */
class BraccioArm
{
public:
  /**
   * @brief Contructor
   */
  BraccioArm();

  /**
   * @brief Initialise the Braccio arm to a straight up position
   */
  void init();

  /**
   * @brief To set the target of a specific servo of the arm
   * @param servo Which servo should be set
   * @param pwm To what PWM the servo should go. The value of this number depends on the contraints of the arm used
   * @param ms In how many milliseconds the arm should reach the position. In reality this number can vary based on
   * different hardware contraints
   * @return True if the target is succesfully set, false if not
   */
  bool setTarget(braccio::Servos servo, uint16_t pwm, uint16_t ms);

  /**
   * @brief Stops a specific servo straight away
   * @param servo The servo which needs to be stopped
   */
  void stopServo(braccio::Servos servo);

  /**
   * @brief Updates the position of each servo
   */
  void update();

private:
  ServoControl servos[DOFS];     /*!< The servo's which are connected*/
  uint16_t constraints[DOFS][2]; /*!< The constraints which are the min and max PWM per servo */

  /**
   * @brief Adds hard coded contraints into the contraints list.
   */
  void initConstraints();
};

#endif  // BRACCIOARM_HPP_
