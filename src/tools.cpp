#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {
  /**
   * Compute RMSE
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd rmse_precal(4);
  rmse_precal << 0,0,0,0;
  unsigned int n = estimations.size();

  // the input list of estimations

  for(unsigned int i=0; i < estimations.size(); ++i)
  {
  
    VectorXd est_gnd_truth_dt = estimations[i] - ground_truth[i];
      est_gnd_truth_dt = est_gnd_truth_dt.array()*est_gnd_truth_dt.array();
      rmse_precal += est_gnd_truth_dt;
    
  }

  rmse = rmse_precal / n;
  rmse = rmse.array().sqrt();
  
  return rmse;
  // call the CalculateRMSE and print out the result
  //cout << CalculateRMSE(estimations, ground_truth) << endl;
  //return 0;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
