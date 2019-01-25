#ifndef SERVICES_H
#define SERVICES_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "HigherLevelInterface.hpp"
#include "braccio_interface/MoveServoToPos.h"
#include "braccio_interface/MoveToPosition.h"

// The namespace of the HigherLevelInterface variable and the service
namespace services
{
void initHLI(const std::string& configFile);

/**
 * Description: This function is meant to return the HLI variable from the cpp file
 * Return: It returns the HLI variable which is declared in the cpp file
 */
HigherLevelInterface& getHLI();

/**
 * Description: this is the method the service MoveServoTo
 */
void moveServo(const braccio_interface::MoveServoToPosConstPtr& msg);

/**
 * Description: this is the method the service MoveToPosition
 */
void moveToPosition(const braccio_interface::MoveToPositionConstPtr& msg);

/**
 * Description: this is the method the service s
 */
void emergencyStop(const std_msgs::EmptyConstPtr&);

};  // namespace services

#endif  // SERVICES_H
