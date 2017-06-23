#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

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
   TODO:
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   TODO:
   * update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_; // error calculation
  KF(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   TODO:
   * update the state by using Extended Kalman Filter equations
   */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);

  // The Forum entry "Radar causes RMSE to go through the roof"
  // fixed the problem that the RMSE was way to hight
  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  while (y(1)> M_PI) {
    y(1) -= 2 * M_PI;
  }

  while (y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }

  KF(y);
}


void KalmanFilter::KF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
