#include "HigherLevelInterface.hpp"

#include <ros/ros.h>

HigherLevelInterface::HigherLevelInterface(const std::string& configFile)
{
  config.open(configFile);
  config.extract();
  std::vector<std::string> usbPort;
  config.findVariable("USBPORT", usbPort);
  std::vector<int32_t> baudRate;
  config.findVariable("BAUDRATE", baudRate);
  hardware_interface = HardwareInterface(usbPort.at(0).c_str(), baudRate.at(0));
}

HigherLevelInterface::~HigherLevelInterface()
{
}

int16_t HigherLevelInterface::moveServoTime(SERVO_TYPE servo, int16_t angle, double time /*sec*/)
{
  ServoInfo test = this->getServoInfo(servo);
  if (angle < test.minDegree || angle > test.maxDegree)
  {
    ROS_ERROR("Angle out of bounds %i < %i || %i >%i", angle, test.minDegree, angle, test.maxDegree);
    return 0;
  }
  hardware_interface.turnServo(servo,
                               this->degreesToPWM(angle, test.minDegree, test.maxDegree, test.minPwm, test.maxPwm),
                               (uint32_t)(time * 1000));
  return angle;
}

void HigherLevelInterface::addDefaultServos()
{
  for (size_t i = 0; i < NR_OF_SERVOS; ++i)
  {
    std::string servo = SERVO_TYPE_TO_STRING.find((SERVO_TYPE)i)->second;
    std::vector<int32_t> servoInfo;
    config.findVariable(servo, servoInfo);

    ROS_INFO("%s:%i,%i,%i,%i", servo.c_str(), servoInfo[0], servoInfo[1], servoInfo[2], servoInfo[3]);
    addServo((SERVO_TYPE)i, (int16_t)servoInfo[0], (int16_t)servoInfo[1], (int16_t)servoInfo[2], (int16_t)servoInfo[3]);
  }
}

void HigherLevelInterface::executePosition(const std::string& position, const double time)
{
  std::vector<int32_t> servoPositions;
  config.findVariable(position, servoPositions);
  for (size_t i = 0; i < servoPositions.size(); ++i)
  {
    moveServoTime((SERVO_TYPE)i, (int16_t)servoPositions.at(i), time);
  }
}

uint32_t HigherLevelInterface::degreesToPWM(int32_t degrees, int32_t minDegree, int32_t maxDegree, uint16_t minPwm,
                                            uint16_t maxPwm)
{
  if (minDegree != 0)
  {
    degrees -= (int32_t)minDegree;
    maxDegree -= (int32_t)minDegree;
    minDegree -= (int32_t)minDegree;
  }

  if (maxDegree > minDegree)
  {
    uint32_t PWM = 0;
    int32_t differenceBetweenDegrees = std::abs(maxDegree - minDegree);

    double PWMperDegree = (maxPwm - minPwm) / (double)differenceBetweenDegrees;
    PWM = (uint32_t)(std::abs(degrees) * PWMperDegree) + minPwm;
    return PWM;
  }
  throw std::invalid_argument("Maxdegree < minDegree");
  return 0;
}

void HigherLevelInterface::addServo(SERVO_TYPE servoType, int16_t minDegree, int16_t maxDegree, uint16_t minPwm,
                                    uint16_t maxPwm)
{
  if (minDegree >= maxDegree)
  {
    throw std::invalid_argument("minDegree can't be equal or bigger than maxDegree");
  }
  try
  {
    getServoInfo(servoType);
  }
  catch (const std::invalid_argument& e)
  {
    ServoInfo newServo;
    newServo.servoType = servoType;
    newServo.minDegree = minDegree;
    newServo.maxDegree = maxDegree;
    newServo.minPwm = minPwm;
    newServo.maxPwm = maxPwm;
    servoInfo.push_back(newServo);
    return;
  }
  throw std::invalid_argument("Servo already exists");
}

void HigherLevelInterface::emergencyStop()
{
  hardware_interface.stop();
}

ServoInfo HigherLevelInterface::getServoInfo(SERVO_TYPE servoType)
{
  for (uint8_t i = 0; i < servoInfo.size(); ++i)
  {
    if (servoInfo.at(i).servoType == servoType)
    {
      return servoInfo.at(servoType);
    }
  }
  throw std::invalid_argument("Servo " + SERVO_TYPE_TO_STRING.find(servoType)->second + " not found");
}
