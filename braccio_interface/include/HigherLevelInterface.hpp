#ifndef HIGHER_LEVEL_INTERFACE_H_
#define HIGHER_LEVEL_INTERFACE_H_

#include <stdint.h>
#include <vector>
#include <chrono>
#include <cmath>

#include "HardwareInterface.hpp"
#include "ServoInfo.hpp"
#include "File.hpp"

/**
 * @class HigherLevelInterface
 * @brief A class to set the arm to certain positions and move servos
 */
class HigherLevelInterface
{
public:
  /**
   * @brief The default constructor
   */
  HigherLevelInterface() = default;

  /**
   * @brief The constructor with a specific configuration file
   * @param config The configuration file of the servos
   */
  HigherLevelInterface(const std::string& config);

  /**
   * @brief The default destructor
   */
  ~HigherLevelInterface();

  /**
   * @brief This moves the servo to an angle within an amount of time
   * @param servo This is an enum for the servo. This is the servo to which the command is send
   * @param angle This is the angle in degrees for the servo
   * @param time This is the time in seconds at which the command should reach his endpoint
   * @param setTimer This can be turned on and off to set the timer and print the states
   * @return This returns the angle at what the servo is send to.
   */
  int16_t moveServoTime(SERVO_TYPE servo, int16_t angle, double time /*sec*/);

  /**
   * @brief This function adds a servo to the servo list with the given parameters
   * @param servoType This is the servo name at which the servo will be added for more info check servo_info.h
   * @param minDegree This is the safe minimum degree of the servo
   * @param maxDegree This is the safe maximum degree of the servo
   */
  void addServo(SERVO_TYPE servoType, int16_t minDegree, int16_t maxDegree, uint16_t minPwm, uint16_t maxPwm);

  /**
   * @brief This will cause all the servo's to stop immediately
   */
  void emergencyStop();

  /**
   * @brief This will add the servos based on the configuration file
   */
  void addDefaultServos();

  /**
   * @brief This will execute a preprogrammed position at a certain time
   * @param position This is the name of the preprogrammed position which are in the config file
   * @param time The time at which the position should reach his final position
   */
  void executePosition(const std::string& position, const double time);

  /**
   * @brief Get the min and max degree of a specific servo
   * @param servoType The servo you want the information from
   * @return This returns the struct ServoInfo with the min and max degree
   */
  ServoInfo getServoInfo(SERVO_TYPE servoType);

private:
  /**
   * @brief This converts the degrees to PWM
   * @param degrees The degrees you want to convert to pwm signal
   * @param minDegree The minimum degree of the servo
   * @param maxDegree The maximum degree of the servo
   * @return The calculated PWM from the degrees
   */
  uint32_t degreesToPWM(int32_t degrees, int32_t minDegree, int32_t maxDegree, uint16_t minPwm, uint16_t maxPwm);

  std::vector<ServoInfo> servoInfo;     /*!< The configuration for each servo */
  HardwareInterface hardware_interface; /*!< The hardware interface which writes the commands to the arm */
  File config;                          /*!< The configuration file for the servos, USB port and baudrate */
};

#endif  // HIGHER_LEVEL_INTERFACE_H_
