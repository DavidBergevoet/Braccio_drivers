#ifndef SERVO_INFO_H_
#define SERVO_INFO_H_
#include <stdint.h>
#include <boost/assign/list_of.hpp>
#include <map>

#define NR_OF_SERVOS 6

/**
 * @enum SERVO_TYPE
 * @brief A enumeration to keep track of all the DOF's of the arm
 */
enum SERVO_TYPE
{
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2,
  WRIST = 3,
  WRISTROTATE = 4,
  GRIPPER = 5
};

/**
 * @brief This is used to convert a SERVO_TYPE enum to string
 * @usage SERVO_TYPE_TO_STRING.find(string)->second;
 */
const static std::map<SERVO_TYPE, std::string> SERVO_TYPE_TO_STRING =
    boost::assign::map_list_of(SERVO_TYPE::BASE, "Base")(SERVO_TYPE::SHOULDER, "Shoulder")(SERVO_TYPE::ELBOW, "Elbow")(
        SERVO_TYPE::WRIST, "Wrist")(SERVO_TYPE::GRIPPER, "Gripper")(SERVO_TYPE::WRISTROTATE, "Wrist_rotate");

/**
 * @struct ServoInfo
 * @brief The enumeration to keep track of all the servo info
 */
struct ServoInfo
{
  SERVO_TYPE servoType; /*!< The servo type of the servo */
  int16_t minDegree;    /*!< The minimum degree the servo can turn */
  int16_t maxDegree;    /*!< The maximum degree the servo can turn */
  uint16_t minPwm;      /*!< The minimum pwm signal of the servo in the minimum degree */
  uint16_t maxPwm;      /*!< The maximum pwm signal of the servo in the maximum degree */
};
#endif
