/*
 * Kinematics.hpp
 *
 *  Created on: Oct 30, 2018
 *      Author: root
 */

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <iostream>
#include <cmath>

#include "Matrix.hpp"

class Kinematics
{
public:
  Kinematics(double l1, double l2, double l3, double beta = 0.1);
  ~Kinematics() = default;

  Kinematics(const Kinematics& other) = delete;
  Kinematics& operator=(const Kinematics& rhs) = delete;
  /*
   * @brief This is the inverse kinematics function whicht calculates the angles that will end up at the goal
   * @param
   * aTheta: this is the theta at what the robotarm starts. This is a matrix with 3 DOF
   * g: This is the goal of the robotarm. This is a matrix with a x and y position
   * origin: this is the translation of the robotarm. This is a matrix with a x and y position.
   * boundaries: This is to bound the DOF's to a min and max angle. This is a matrix with a min and max degree for each
   * DOF
   * @return This returns the new theta's for the goal
   */
  Matrix<double, 3, 1> inverseKinematics(const Matrix<double, 2, 1>& g, const Matrix<double, 2, 1>& origin,
                                         const Matrix<double, 3, 2>& boundaries);

  /*
   * @brief This is the function to get the end point for the given theta's
   * @param
   * theta: These are the angles of the DOF. This is a matrix for 3 angles
   * origin: This is the translation of the robotarm. This is a matrix with a x and y position.
   * @return This returns a matrix with a x and y position of the end point
   */
  Matrix<double, 2, 1> forwardKinematics(const Matrix<double, 3, 1>& theta, const Matrix<double, 2, 1>& origin);

  /*
   * @brief This is a function to get the degree at what the base has to turn to face the cube.
   * @param
   * x: This is the x of the point what the robot should face
   * y: This is the y of the point what the robot should face
   * r_x: This is the x of the translation of the robot
   * r_y: This is the y of the translation of the robot
   * returnV: This is a reference to a double value at what this function returns the C of pythagoras
   * @return This returns the angle at what the base should turn
   */
  int16_t getBaseDegree(double x, double y, double r_x, double r_y, double& returnV);

  /*
   * @brief Set the lengths of the arm
   */
  void setLengths(double l1, double l2, double l3);

private:
  double l1, l2, l3;
  double beta;
  Matrix<double, 3, 1> currTheta;

  /*
   * @brief This is a function to get a jacobian matrix for the given theta's
   * @param
   * theta: These are the theta's of the arm over what the jacobian matrix should be calculated
   * @return This returns the jacobian matrix in a matrix
   */
  Matrix<double, 2, 3> getJacobi(const Matrix<double, 3, 1>& theta);
};
#endif /* KINEMATICS_HPP_ */
