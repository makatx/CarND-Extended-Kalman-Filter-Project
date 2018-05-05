#include "kalman_filter.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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
  x_ = F_ * x_ ;
  P_ = F_*P_*F_.transpose() + Q_ ;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(2,2);
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd hx = VectorXd(3);
  hx(0) = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  hx(1) = atan2(x_(1), x_(0));
  hx(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / hx(0);

  VectorXd y = z - hx;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ = x_ + K*y;
  // VectorXd x_polar = x_ + K*y;
  //
  // float rho = x_polar(0);
  // float phi = x_polar(1);
  // float phi_dot = x_polar(3)
  // x_(0) =  rho * cos(phi);
  // x_(1) = rho * sin(phi);
  // x_(2) = phi_dot * cos(phi);
  // x_(3) = phi_dot * sin(phi);


  MatrixXd I = MatrixXd::Identity(3,3);
  P_ = (I - K*H_)*P_;
}
