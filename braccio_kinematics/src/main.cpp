#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>

#include <geometry_msgs/Point.h>
#include <braccio_interface/MoveServoToPos.h>

#include "Kinematics.hpp"

#define BUFFER_SIZE 10
#define DURATION 5.0

ros::Publisher* pub = nullptr;
Kinematics k(0, 0, 0);
Matrix<double, 2, 1> origin{ { { 0 } }, { { 0 } } };
Matrix<double, 3, 2> bounds{ { { -60 }, { 40 } }, { { -70 }, { 90 } }, { { -90 }, { 90 } } };

void calculateMotion(const geometry_msgs::PointConstPtr& msg)
{
  braccio_interface::MoveServoToPos outputMsg;
  double hypotenuse = 0;
  outputMsg.servoNr = 0;
  outputMsg.servoPos = (int64_t)k.getBaseDegree(msg->x, msg->y, origin.at(0, 0), origin.at(1, 0), hypotenuse);
  outputMsg.duration = DURATION;
  pub->publish(outputMsg);
  std::cout << "Pos " << outputMsg.servoPos << std::endl;
  ros::spinOnce();

  Matrix<double, 2, 1> goal{ { { hypotenuse } }, { { msg->z } } };
  Matrix<double, 3, 1> angles = k.inverseKinematics(goal, origin, bounds);
  std::cout << angles << std::endl;

  for (size_t i = 0; i < 3; ++i)
  {
    outputMsg.servoNr = (int64_t)i + 1;
    outputMsg.servoPos = (int64_t)angles.at(i, 0);
    outputMsg.duration = DURATION;
    pub->publish(outputMsg);
    ros::spinOnce();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Kinematics");
  ros::NodeHandle node("~");
  ros::Publisher tempPub = node.advertise<braccio_interface::MoveServoToPos>("/braccio/servo", BUFFER_SIZE);
  pub = &tempPub;
  double l1 = 0;
  node.getParam("l1", l1);
  double l2 = 0;
  node.getParam("l2", l2);
  double l3 = 0;
  node.getParam("l3", l3);

  k.setLengths(l1, l2, l3);

  ros::Subscriber kinSub = node.subscribe("/position", BUFFER_SIZE, calculateMotion);

  ros::spin();
}
