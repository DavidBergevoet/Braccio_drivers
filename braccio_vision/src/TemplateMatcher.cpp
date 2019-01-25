#include "TemplateMatcher.hpp"

#include <ros/ros.h>

#define IMG_EXTENTION ".png"

TemplateMatcher::TemplateMatcher(const std::string& templatePath, uint8_t nrOfTemplates) : templatePath(templatePath)
{
  templates.resize(nrOfTemplates);
}

void TemplateMatcher::loadTemplates()
{
  for (size_t i = 0; i < templates.size(); ++i)
  {
    templates.at(i) = Template(templatePath + std::to_string(i) + std::string(IMG_EXTENTION));
  }
}
cv::Mat debugImg;
std::vector<cv::Point> TemplateMatcher::getObjects(const cv::Mat& inputImg, bool debugMode /*=false*/) const
{
  cv::Mat edges;
  debugImg = inputImg;
  edgeDetect(inputImg, edges);
  std::vector<std::vector<cv::Point>> contours;
  getShapes(edges, contours, debugImg);

  std::vector<cv::Point> returnVec;

  matchShapes(contours, returnVec);
  cv::namedWindow("edges", cv::WINDOW_NORMAL);

  cv::resizeWindow("edges", 800, 600);
  cv::imshow("edges", debugImg);

  return returnVec;
}

void TemplateMatcher::edgeDetect(const cv::Mat& src, cv::Mat& dst) const
{
  cv::Mat tempImg = src;
  cv::Mat biFilter;
  cv::Mat cannyImg;

//  cv::bilateralFilter(tempImg, biFilter, -1, 20, 20);
//  cv::GaussianBlur(biFilter, biFilter, cv::Size(9, 9), 2, 2);
  //    while(ros::ok()){
  //  	  cv::imshow("b4",tempImg);
  //  	  cv::imshow("af",biFilter);
  //  	  cv::waitKey(1);
  //    }

  cv::Canny(tempImg, dst, 0, 60);
//  cv::dilate(cannyImg, dst, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
}

#define MIN_CONTOUR_SIZE 5000

void TemplateMatcher::getShapes(const cv::Mat& edges, std::vector<std::vector<cv::Point>>& dst, cv::Mat& debugImg) const
{
  std::vector<cv::Vec4i> hierarchy;
  findContours(edges, dst, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < dst.size(); ++i)
  {
    if (cv::contourArea(dst.at(i)) < MIN_CONTOUR_SIZE)  // Do initial contour filtering
    {
      dst.erase(dst.begin() + i);
      i--;
    }
  }
  for (size_t i = 0; i < dst.size(); ++i)
  {
    drawContours(debugImg, dst, i, cv::Scalar(0, 0, 255), 10, 8, hierarchy, 0, cv::Point());
  }
}

void TemplateMatcher::matchShapes(const std::vector<std::vector<cv::Point>>& contours,
                                  std::vector<cv::Point>& dst) const
{
  cv::Point position;
  for (size_t c = 0; c < contours.size(); ++c)
  {
    for (size_t t = 0; t < templates.size(); ++t)
    {
      ROS_INFO("match:%zu,%f", c, templates.at(t).matchShapes(contours.at(c)));
      float matchNr = templates.at(t).matchShapes(contours.at(c));
      if (matchNr < 2 /* && matchNr >5*/)
      {
        std::vector<cv::Vec4i> hierarchy;
        drawContours(debugImg, contours, c, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0, cv::Point());
        cv::putText(debugImg, std::to_string(cv::contourArea(contours.at(c))), cv::Point(100, 100),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));
        cv::Moments m = cv::moments(contours.at(c), false);

        position.x = m.m10 / m.m00;
        position.y = m.m01 / m.m00;

        dst.push_back(position);
        break;
      }
    }
  }
}
