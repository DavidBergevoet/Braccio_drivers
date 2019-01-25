#include "Services.hpp"
HigherLevelInterface* HLI(new HigherLevelInterface());

void services::initHLI(const std::string& configFile)
{
  HLI = (new HigherLevelInterface(configFile));
}

HigherLevelInterface& services::getHLI()
{
  return *HLI;
}

void services::moveServo(const braccio_interface::MoveServoToPosConstPtr& msg)
{
  HLI->moveServoTime((SERVO_TYPE)msg->servoNr, (int16_t)msg->servoPos, msg->duration);
}

void services::moveToPosition(const braccio_interface::MoveToPositionConstPtr& msg)
{
  HLI->executePosition(msg->position, msg->duration);
}

void services::emergencyStop(const std_msgs::EmptyConstPtr&)
{
  HLI->emergencyStop();
  ROS_WARN("EMERGENCY STOP INITIATED");
}
