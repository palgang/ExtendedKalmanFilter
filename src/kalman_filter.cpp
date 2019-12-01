#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}



void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  cout << "z_pred calculated" << endl;
  VectorXd y = z - z_pred;
  cout << "Xd calculated" << endl;
  MatrixXd Ht = H_.transpose();
  /*cout << "Ht calculated" << endl;
  cout << "H_ = " << H_ << endl;
  cout << "P_ = " << P_ << endl;
  cout << "Ht_ = " << Ht << endl;
  cout << "R_ = " << R_ << endl;'*/
  MatrixXd S = H_ * P_ * Ht + R_;
  //cout << "S calculated" << endl;
  MatrixXd Si = S.inverse();
  //cout << "Si calculated" << endl;
  MatrixXd PHt = P_ * Ht;
  //cout << "PHt calculated" << endl;
  MatrixXd K = PHt * Si;
  //cout << "K calculated" << endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   * H matrix in Kalman filters will be replaced by Hj (Jacobian)
    To calculate y the h function is used instead of the H matrix.
    To calculate x​′​​, the prediction update function, f, is used instead of the F matrix.
    The F matrix will be replaced by F​j​​ when calculating P​′​​.
   */
    // used to compute the RMSE later
  cout << "Start UpdateEKF" << endl;
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  VectorXd z_pred = VectorXd(3);
  z_pred(0) = sqrt(pow(px,2)+pow(py,2));
  //z_pred(1) = atan(py/px);
  z_pred(1) = atan2(py,px);
  z_pred(2) = (px*vx + py*vy)/(sqrt(pow(px,2)+pow(py,2)));                    
  cout << "z_pred calculated" << endl;
  VectorXd y = z - z_pred;
  cout << "y calculated" << endl;
  while ((y(1) < -M_PI) || y(1) > M_PI ){
    if (y(1) < -M_PI){
      y(1) = y(1) + 2*M_PI;
    }else{
      y(1) = y(1) - 2*M_PI;
    }
    
    cout << "y Angle: " << y(1)<<endl;
  }
  MatrixXd Ht = Hj.transpose();

  MatrixXd S = Hj * P_ * Ht + R_;
  //cout << "S calculated" << endl;
  MatrixXd Si = S.inverse();
  //cout << "Si calculated" << endl;
  MatrixXd PHt = P_ * Ht;
  //cout << "PHt calculated" << endl;
  MatrixXd K = PHt * Si;
  //cout << "K calculated" << endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;

}
