#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {
  // set tolerance small enough
   _TOL_0_ = 0.000001;
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  VectorXd rmse = VectorXd(4);
  rmse << 0,0,0,0;

  size_t num_samples = estimations.size();

  // input data pre checking
  if (num_samples == 0) {
     cout << "ERROR: estimations size is zero. Using default RMSE." << endl;
     return rmse;
  }
  if (num_samples != ground_truth.size()) {
     cout << "ERROR: estimations size not matching ground_truth. Using default RMSE." << endl;
     return rmse;
  }

  // RMSE
  for (size_t s = 0; s < num_samples; s++) {
    VectorXd residual = estimations[s] - ground_truth[s];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / num_samples;
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  double p_x = x_state(0);
  double p_y = x_state(1);
  double v_x = x_state(2);
  double v_y = x_state(3);

  double p_xy_2 = p_x * p_x + p_y * p_y;
  
  MatrixXd Hj = MatrixXd(3, 4);
  Hj << 1, 1, 0, 0,
        1, 1, 0, 0,
        1, 1, 1, 1;
  
  // avoid divide by zero
  if (p_xy_2 < _TOL_0_) {
     cout << "ERROR: Divide by zero detected. Using default Jacobians." << endl;
     return Hj;
  }

  double p_xy   = sqrt(p_xy_2);
  double p_xy_3 = p_xy * p_xy_2;

  Hj <<                             p_x / p_xy,                               p_y / p_xy,            0,            0,
                                 -p_y / p_xy_2,                             p_x / p_xy_2,            0,            0,
        p_y * (v_x * p_y - v_y * p_x) / p_xy_3,   p_x * (v_y * p_x - v_x * p_y) / p_xy_3,   p_x / p_xy,   p_y / p_xy;

  return Hj;
}
