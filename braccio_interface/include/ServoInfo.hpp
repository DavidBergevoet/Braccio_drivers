#ifndef SERVO_INFO_H_
#define SERVO_INFO_H_
#include <stdint.h>
#include <boost/assign/list_of.hpp>
#include <map>

#define NR_OF_SERVOS 6
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

struct ServoInfo
{
  SERVO_TYPE servoType;
  int16_t minDegree;
  int16_t maxDegree;
  uint16_t minPwm;
  uint16_t maxPwm;
};
#endif
