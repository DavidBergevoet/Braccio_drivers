#ifndef SERVICES_H
#define SERVICES_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "HigherLevelInterface.hpp"
#include "braccio_interface/MoveServoToPos.h"
#include "braccio_interface/MoveToPosition.h"

/**
 * @namespace services
 * @brief The namespace for the callback functions and Higher Level interface instance
 */
namespace services
{
/**
 * @brief This will initialise the Higher level interface with the configfile
 * @param configFile The configuration file for the higher level interface
 */
void initHLI(const std::string& configFile);

/**
 * @brief This function is meant to return the HLI variable from the cpp file
 * @return It returns the HLI variable which is declared in the cpp file
 */
HigherLevelInterface& getHLI();

/**
 * @brief This is the callback for moving a specific servo to a specific position in degrees
 * @param msg The incoming message for moving a servo
 */
void moveServo(const braccio_interface::MoveServoToPosConstPtr& msg);

/**
 * @brief This is the callback for moving the arm to a specific position
 * @param msg The message which is parsed and has the variables to move the arm
 */
void moveToPosition(const braccio_interface::MoveToPositionConstPtr& msg);

/**
 * @brief This is the callback to stop the entire arm from moving
 */
void emergencyStop(const std_msgs::EmptyConstPtr&);

};  // namespace services

#endif  // SERVICES_H
