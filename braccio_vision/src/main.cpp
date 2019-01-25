#include <iostream>
#include <ros/ros.h>
#include "TemplateMatcher.hpp"

#define BACKGROUND_WINDOW "background"

cv::Ptr<cv::BackgroundSubtractor> pMOG2;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "braccio_vision");

  if (argc < 3)
  {
    ROS_FATAL("Arg 1 is the path to the templates folder\nArg 2 is the nr of templates available");
  }
  cv::VideoCapture capture(0);
  if (!capture.isOpened())
  {
    ROS_FATAL("Camera: %i cant be opened", 0);
    return -1;
  }
  pMOG2 = cv::createBackgroundSubtractorMOG2();

  char c = 0;
  cv::Mat backgroundFrame;
  cv::Mat foregroundFrame;

  ROS_INFO("To set the background image press esc");
  cv::namedWindow(BACKGROUND_WINDOW, cv::WINDOW_NORMAL);
  cv::resizeWindow(BACKGROUND_WINDOW, 400, 300);
  while (c != 27)
  {
    capture.read(backgroundFrame);
    cv::imshow(BACKGROUND_WINDOW, backgroundFrame);
    c = cv::waitKey(1);
  }
  cv::destroyWindow(BACKGROUND_WINDOW);
  pMOG2->apply(backgroundFrame, foregroundFrame);
  ROS_INFO("To compare the background press esc");
  cv::namedWindow(BACKGROUND_WINDOW, cv::WINDOW_NORMAL);
  cv::resizeWindow(BACKGROUND_WINDOW, 400, 300);
  c = 0;

  while (c != 27)
  {
    capture.read(foregroundFrame);
    cv::imshow(BACKGROUND_WINDOW, foregroundFrame);
    c = cv::waitKey(1);
  }
  cv::destroyWindow(BACKGROUND_WINDOW);
  cv::Mat outputFrame;

  c = 0;
  cv::namedWindow("output", cv::WINDOW_NORMAL);
  cv::resizeWindow("output", 400, 300);
  TemplateMatcher tempMatcher(argv[1], (uint8_t)std::stoi(argv[2]));
  while (c != 27)
  {
    capture.read(foregroundFrame);
    pMOG2->apply(foregroundFrame, outputFrame);
    cv::erode(outputFrame, outputFrame, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(outputFrame, outputFrame, cv::Mat(), cv::Point(-1, -1), 2);
    tempMatcher.getObjects(outputFrame, true);

    cv::imshow("output", outputFrame);
    c = cv::waitKey(1);
  }

  //  TemplateMatcher tempMatcher(argv[1], (uint8_t)std::stoi(argv[2]));
  //  tempMatcher.loadTemplates();
  //  std::string imgPath(argv[1]);
  //  imgPath += "handMy.jpg";
  //  tempMatcher.getObjects(cv::imread(imgPath), true);
  capture.release();
  return 0;
}
