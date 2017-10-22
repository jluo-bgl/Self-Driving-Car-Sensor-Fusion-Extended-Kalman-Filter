#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd temp(4);
  temp << 1, 2, 3, 4;
  return temp;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd temp(3,4);
  return temp;
}
