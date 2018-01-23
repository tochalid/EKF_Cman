#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// constructor and deconstructor
Tools::Tools() {}
Tools::~Tools() {}

// REVIEW: RMSE calculation
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // add up squared residual
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean and squared root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

// REVIEW: Jacobian calculation
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-calcuate terms for multiple use
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // check if division by zero
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // calculate Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0, 
  -(py / c1), (px / c1), 0, 0,
  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
