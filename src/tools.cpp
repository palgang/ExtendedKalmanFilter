#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */  
  cout << "Entering Tools.CalculateRMSE " << endl;
  /*cout << "estimations size: " << endl;
  cout << estimations.size() << endl;
  cout << "ground_truth size: " << endl;
  cout << ground_truth.size() << endl;'*/

  VectorXd rmse(4);
  rmse << 0,0,0,0;


  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
      }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    //cout << "before residual " << endl;

    //cout << "estimations[i]: " << endl;
    //cout << estimations[i] << endl;
    VectorXd residual = estimations[i] - ground_truth[i];
    //cout << "Residual: " << endl;
    //cout << residual << endl;

    // coefficient-wise multiplication
    //cout << "residual: " << residual << endl;
    residual = residual.array()*residual.array();
    //cout << "residual2: " << residual << endl;
    rmse += residual;
  }
  //cout << "rmse (residual): " << rmse << endl;
  //cout << "estimations.size(): " << estimations.size() << endl;

  // calculate the mean
  rmse = rmse/estimations.size();
  //cout << "rmse (divided by estimations.size()): " << rmse << endl;
  // calculate the squared root
  rmse = rmse.array().sqrt();

  //cout << "return rmse: " << rmse << endl;

  // return the result
  return rmse;   
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if ((px == 0) && (py == 0)){
      cout << "Error: both px and py are 0" << endl;
  }
  // compute the Jacobian matrix
  else{
      Hj(0,0) = px / (sqrt(pow(px,2)+pow(py,2)));
      Hj(0,1) = py / (sqrt(pow(px,2)+pow(py,2)));
      Hj(0,2) = 0;
      Hj(0,3) = 0;

      Hj(1,0) = - py / (pow(px,2) + pow(py,2));
      Hj(1,1) = px / (pow(px,2) + pow(py,2));
      Hj(1,2) = 0;
      Hj(1,3) = 0;

      Hj(2,0) = (py*(vx*py - vy*px)) / pow(pow(px,2) + pow(py,2), 3.0/2.0);
      Hj(2,1) = (px*(vy*px - vx*py)) / pow(pow(px,2) + pow(py,2), 3.0/2.0);
      Hj(2,2) = px / (sqrt(pow(px,2)+pow(py,2)));
      Hj(2,3) = py / (sqrt(pow(px,2)+pow(py,2)));
  }
  


  return Hj;   
}
