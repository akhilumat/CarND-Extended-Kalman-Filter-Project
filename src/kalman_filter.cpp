#include "kalman_filter.h"
#include<cmath>
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;
// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.
VectorXd ekfpredict(VectorXd x);
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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  /*  * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

//new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

VectorXd ekfpredict(VectorXd x){
  VectorXd z = VectorXd(3);
  z << 0,0,0;
  double px = x[0];
  double py = x[1];
  double vx = x[2];
  double vy = x[3];
  double c1 = sqrt(px*px + py*py);
  double c2 = atan2(py,px);
  double c3 = (px*vx + py*vy)/c1 ;
if(c1!=0)
  {z[0]= c1;
  z[1] = c2;
  z[2]= c3;}
  return z;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = ekfpredict(x_) ;
  VectorXd y = z - z_pred;
  y[1]= atan2(sin(y[1]),cos(y[1]));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

//new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
