#ifndef TEMPLATE_HPP
#define TEMPLATE_HPP

#include <vector>

#include <opencv2/opencv.hpp>

#define MATCH_METHOD CV_CONTOURS_MATCH_I2

/**
 * @brief Class that stores the cup patterns
 */
class Template
{
public:
  Template() = default;
  Template(const Template& other);
  virtual ~Template() = default;

  /**
   * @brief Constructor that loads the template from an image
   * @param imagePath: The path of the template image
   */
  Template(const std::string& imagePath);

  /**
   * @brief Loads the template with an image
   * @param imagePath: The path of the template image
   */
  void loadImage(const std::string& imagePath);

  /**
   * @brief Matches the shape of a potential cup to this template
   * @param cupContours: The contours of the potential cup
   * @return The match of the cup, values below 0.05 mean a high similarity
   */
  double matchShapes(const std::vector<cv::Point>& cupContours) const;

private:
  /**
   * @brief The contours of this template
   */
  std::vector<cv::Point> contours;
};

#endif
