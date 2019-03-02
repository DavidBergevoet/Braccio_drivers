#ifndef SERVOCONTROL_HPP_
#define SERVOCONTROL_HPP_

#include <Arduino.h>
#include <Servo.h>

#define STEP_TIME 5  // ms

/**
 * @brief This class represents a single servo. This can be moved and stopped.
 */
class ServoControl
{
public:
  /**
   * @brief Empty constructor (default)
   */
  ServoControl();

  /**
   * @brief Contructor with a pin number
   * @param pin The pin number where the servo is connected to
   */
  ServoControl(uint8_t pin);

  /**
   * @brief This will set the target of the servo with the time which it takes to get there
   * @param pwm The desired position of the servo
   * @param ms The time which the servo needs to reach that position
   * @return True if the target has been set succesfully, false if not
   */
  bool setTarget(uint16_t pwm, uint16_t ms);

  /**
   * @brief This will update the position of the servo based on the time to travel
   */
  void updatePos();

  /**
   * @brief A getter for the current position
   * @return The current position of the servo in PWM
   */
  uint16_t getCurPosition() const;

private:
  uint16_t curPosition;    /*!< The current position of the servo in PWM */
  bool initialised;        /*!< This will check wheter or not a pin number is given to the class */
  Servo servo;             /*!< The servo class which is written to. This is a class embedded in Arduino */
  uint16_t stepSize;       /*!< The calculated stepsize which the servo takes at a certain delay */
  uint16_t target;         /*!< The target of the servo. This is set by the setTarget function */
  uint32_t previousUpdate; /*!< This is the number of milliseconds at which the previous step has taken place */
};
#endif  // SERVOCONTROL_HPP_
