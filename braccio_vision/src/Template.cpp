#include "Template.hpp"
#include <ros/ros.h>

Template::Template(const Template& other) : contours(other.contours)
{
}

Template::Template(const std::string& imagePath)
{
  loadImage(imagePath);
}

void Template::loadImage(const std::string& imagePath)
{
  cv::Mat temp = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
  std::vector<std::vector<cv::Point>> con;

  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(temp, con, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  if (con.size() != 1)
  {
    ROS_ERROR("Invalid template at: '%s' Template size:  %zu!= 1", imagePath.c_str(),con.size());
    return;
  }
  ROS_DEBUG("path %s", imagePath.c_str());
  contours = con.at(0);
}

double Template::matchShapes(const std::vector<cv::Point>& cupContours) const
{
  return cv::matchShapes(contours, cupContours, MATCH_METHOD, 0);
}
