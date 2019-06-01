#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::_UpdateRoutine(const VectorXd &y) {
  // measurement update routine common to KF and EKF
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

double KalmanFilter::_AngleWrap(double radian) {
  // radian angle normalized to [-pi, pi)
  double pi_2 = M_PI * 2;
  radian = fmod(radian + M_PI, pi_2);
  if (radian < 0)
    radian += pi_2;

  return radian - M_PI;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  _UpdateRoutine(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  double p_x = x_(0);
  double p_y = x_(1);
  double v_x = x_(2);
  double v_y = x_(3);
  
  // convert from cartesian coordinates to polar coordinates
  double p_xy = sqrt(p_x * p_x + p_y * p_y);
  VectorXd z_pred = VectorXd(z.size());
  z_pred << p_xy, atan2(p_y, p_x), (p_x * v_x + p_y * v_y) / p_xy;
  VectorXd y = z - z_pred;

  // normalise angle
  y(1) = _AngleWrap(y(1));

  _UpdateRoutine(y);
}
