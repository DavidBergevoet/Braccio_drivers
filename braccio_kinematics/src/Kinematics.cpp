#include "Kinematics.hpp"

#include <ros/ros.h>

#define radiansToDegrees(angleRadians) ((angleRadians)*180.0 / M_PI)
#define degreesToRadians(angleDegrees) ((angleDegrees)*M_PI / 180.0)

#define BEGIN_THETA_VALUE 0.1

Kinematics::Kinematics(double l1, double l2, double l3, double beta /*=0.1*/) : l1(l1), l2(l2), l3(l3), beta(beta)
{
  currTheta = Matrix<double, 3, 1>{ { { BEGIN_THETA_VALUE } }, { { BEGIN_THETA_VALUE } }, { { BEGIN_THETA_VALUE } } };
}

Matrix<double, 2, 1> Kinematics::forwardKinematics(const Matrix<double, 3, 1>& theta,
                                                   const Matrix<double, 2, 1>& origin)
{
  double new_x = origin.at(0, 0) + l1 * sin(degreesToRadians(theta[0][0])) +
                 l2 * sin(degreesToRadians(theta[1][0] + theta[0][0])) +
                 l3 * sin(degreesToRadians(theta[2][0] + theta[1][0] + theta[0][0]));
  double new_y = origin.at(1, 0) + l1 * cos(degreesToRadians(theta[0][0])) +
                 l2 * cos(degreesToRadians(theta[1][0] + theta[0][0])) +
                 l3 * cos(degreesToRadians(theta[2][0] + theta[1][0] + theta[0][0]));

  return Matrix<double, 2, 1>{ { { new_x } }, { { new_y } } };
}

int16_t Kinematics::getBaseDegree(double x, double y, double r_x, double r_y, double& returnV)
{
  double d_x = std::abs(x - r_x);
  double d_y = std::abs(y - r_y);

  double c = sqrt(pow(d_x, 2) + pow(d_y, 2));
  returnV = c;

  int16_t alpha = (int16_t)radiansToDegrees(asin((double)d_x / c));

  if (x - r_x < 0)
  {
    alpha = int16_t(alpha * -1);
  }

  return (int16_t)alpha;
}

Matrix<double, 3, 1> Kinematics::inverseKinematics(const Matrix<double, 2, 1>& g, const Matrix<double, 2, 1>& origin,
                                                   const Matrix<double, 3, 2>& boundaries)
{
  Matrix<double, 3, 1> theta = currTheta;
  Matrix<double, 2, 1> e = forwardKinematics(theta, origin);
  uint32_t nrOfSteps = 0;
  double high = 0;

  while (!equals(e, g, std::numeric_limits<double>::epsilon(), 1000000))
  {
    nrOfSteps++;
    if (nrOfSteps > 300000)
    {
      ROS_WARN("Trying again for the %u time", high);
      nrOfSteps = 0;
      high++;
      Matrix<double, 3, 1> add{ { { high * 1 } }, { { high * 1 } }, { { high * 1 } } };
      if (high > 3)
      {
        theta = currTheta - add;
      }
      else
      {
        theta = currTheta + add;
      }
      if (high > 10)
      {
        throw std::runtime_error("Can't find configuration");
      }
    }

    Matrix<double, 2, 3> jacobi = getJacobi(theta);

    Matrix<double, 3, 2> jacobiInverse = (jacobi.transpose() * jacobi).inverse() * jacobi.transpose();

    Matrix<double, 2, 1> d_e = (g - e) * beta;

    Matrix<double, 3, 1> d_theta = jacobiInverse * d_e;

    theta += d_theta;
    for (size_t i = 0; i < 3; ++i)
    {
      if (theta.at(i, 0) < boundaries[i][0])
      {
        theta.at(i, 0) = boundaries[i][0];
      }
      else if (theta.at(i, 0) > boundaries[i][1])
      {
        theta.at(i, 0) = boundaries[i][1];
      }
    }
    e = forwardKinematics(theta, origin);
  }
  currTheta = theta;
  return currTheta;
}

void Kinematics::setLengths(double l1, double l2, double l3)
{
  this->l1 = l1;
  this->l2 = l2;
  this->l3 = l3;
}

Matrix<double, 2, 3> Kinematics::getJacobi(const Matrix<double, 3, 1>& theta)
{
  double x_partial_1 = l1 * cos(degreesToRadians(theta[0][0])) + l2 * cos(degreesToRadians(theta[0][0] + theta[1][0])) +
                       l3 * cos(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));
  double x_partial_2 = l2 * cos(degreesToRadians(theta[0][0] + theta[1][0])) +
                       l3 * cos(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));
  double x_partial_3 = l3 * cos(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));

  double y_partial_1 = l1 * -1 * sin(degreesToRadians(theta[0][0])) +
                       l2 * -1 * sin(degreesToRadians(theta[0][0] + theta[1][0])) +
                       l3 * -1 * sin(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));
  double y_partial_2 = l2 * -1 * sin(degreesToRadians(theta[0][0] + theta[1][0])) +
                       l3 * -1 * sin(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));
  double y_partial_3 = l3 * -1 * sin(degreesToRadians(theta[0][0] + theta[1][0] + theta[2][0]));

  return Matrix<double, 2, 3>{ { x_partial_1, x_partial_2, x_partial_3 }, { y_partial_1, y_partial_2, y_partial_3 } };
}
