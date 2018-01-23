#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_laser_in,
                        MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_radar_ = R_radar_in;
  R_laser_ = R_laser_in;
  Q_ = Q_in;
}

// REVIEW: Predict the state
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

// REVIEW: Update the state using Kalman Filter equations
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// REVIEW: update the state by using Extended Kalman Filter equations
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = VectorXd(3);
  double rho, phi, rho_dot;

  // convert radar measurements from cartesian coordinates (x, y, vx, vy) to
  // polar (rho, phi, rho_dot).
  rho = sqrt(double(x_[0] * x_[0] + x_[1] * x_[1]));
  phi = atan2(double(x_[1]), double(x_[0]));
  if (rho < 0.0001) {
    rho = 0.0001;
  }
  rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;

  // normalize the angle between -pi to pi
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  while (y[1] > M_PI)  // if
    y[1] -= 2.f * M_PI;
  while (y[1] < -M_PI)  // if
    y[1] += 2.f * M_PI;

  // following is exact the same as in the function of KalmanFilter::Update()
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd PHt = P_ * Ht;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}
