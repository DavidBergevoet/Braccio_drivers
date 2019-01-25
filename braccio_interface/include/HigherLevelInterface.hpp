#ifndef HIGHER_LEVEL_INTERFACE_H_
#define HIGHER_LEVEL_INTERFACE_H_

#include <stdint.h>
#include <vector>
#include <chrono>
#include <cmath>

#include "HardwareInterface.hpp"
#include "ServoInfo.hpp"
#include "File.hpp"

class HigherLevelInterface
{
public:
  // Constructor and Destructor
  HigherLevelInterface() = default;
  HigherLevelInterface(const std::string& config);
  ~HigherLevelInterface();

  /**
   * Description: This moves the servo to an angle within an amount of time
   * Parameters
   * servo: This is an enum for the servo. This is the servo to which the command is send
   * angle: This is the angle in degrees for the servo
   * time: This is the time in seconds at which the command should reach his endpoint
   * setTimer: This can be turned on and off to set the timer and print the states
   * Return: This returns the angle at what the servo is send to.
   */
  int16_t moveServoTime(SERVO_TYPE servo, int16_t angle, double time /*sec*/);

  /**
   * Description: This function adds a servo to the servo list with the given parameters
   * Parameters
   * servoType: This is the servo name at which the servo will be added for more info check servo_info.h
   * minDegree: This is the safe minimum degree of the servo
   * maxDegree: This is the safe maximum degree of the servo
   */
  void addServo(SERVO_TYPE servoType, int16_t minDegree, int16_t maxDegree, uint16_t minPwm, uint16_t maxPwm);

  /**
   * Description: This will cause all the servo's to stop immediately
   */
  void emergencyStop();

  /*
   * Description: This will init the default servos for the AL5D arm
   */
  void addDefaultServos();

  /**
   * Description: This will execute a preprogrammed position at a certain time
   * Parameters
   * position: This is the name of the preprogrammed position which are in the config file
   * time: The time at which the position should reach his final position
   * startState: The state at which the robotic arm starts. Mostly MOVING or INITIALISING
   */
  void executePosition(const std::string& position, const double time);

  /**
   * Description: Get the min and max degree of a specific servo
   * Parameters
   * servoType: The servo you want the information from
   * Return: This returns the struct ServoInfo with the min and max degree
   */
  ServoInfo getServoInfo(SERVO_TYPE servoType);

private:
  /**
   * Description: This converts the degrees to PWM
   * Parameters
   * degrees: The degrees you want to convert to pwm signal
   * minDegree: The minimum degree of the servo
   * maxDegree: The maximum degree of the servo
   * Return: The calculated PWM from the degrees
   */
  uint32_t degreesToPWM(int32_t degrees, int32_t minDegree, int32_t maxDegree, uint16_t minPwm, uint16_t maxPwm);

  std::vector<ServoInfo> servoInfo;
  HardwareInterface hardware_interface;
  File config;
};

#endif  // HIGHER_LEVEL_INTERFACE_H_
