#ifndef TEMPLATEMATCHER_HPP
#define TEMPLATEMATCHER_HPP

#include "Template.hpp"

class TemplateMatcher
{
public:
  TemplateMatcher(const std::string& templatePath, uint8_t nrOfTemplates);
  TemplateMatcher(const TemplateMatcher&) = delete;
  ~TemplateMatcher() = default;

  void loadTemplates();
  std::vector<cv::Point> getObjects(const cv::Mat& inputImage, bool debugMode = false) const;

private:
  std::string templatePath;
  std::vector<Template> templates;

  void edgeDetect(const cv::Mat& src, cv::Mat& dst) const;

  void getShapes(const cv::Mat& edges, std::vector<std::vector<cv::Point>>& dst, cv::Mat& debugImg) const;

  void matchShapes(const std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Point>& dst) const;
};

#endif  // TEMPLATEMATCHER_HPP
