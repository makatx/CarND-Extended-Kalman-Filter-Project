#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd(4);
  if (estimations.size() == 0 || ground_truth.size() == 0 || estimations.size()!=ground_truth.size()) {
    cout << "RMSE: Invalid arrays passed" << endl;
    rmse << -1,-1,-1,-1;
    return rmse;
  }
  rmse << 0,0,0,0;
  for(int i=0; i < estimations.size(); ++i){
        VectorXd e = estimations[i];
        VectorXd g = ground_truth[i];
        VectorXd diff = e-g;
        diff = diff.array() * diff.array();
        rmse = diff + rmse;


	}
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE
	Hj << 0,0,0,0,
	      0,0,0,0,
	      0,0,0,0;


	//check division by zero
	// if (px==0 && py==0) {
	//     cout << "Computation by Zero" << endl;
	//     return Hj;
	// }
	//compute the Jacobian matrix
    float hyp = sqrt(px*px + py*py);
    if (hyp==0) {
      hyp = 0.0001;
    }
    float h11 = px / hyp;
    float h12 = py / hyp;
    float h21 = -py / (px*px + py*py);
    float h22 = px / (px*px + py*py);
    float h31 = py * (vx*py-vy*px) / pow((px*px+py*py), 1.5);
    float h32 = px * (vy*px-vx*py) / pow((px*px+py*py), 1.5);


    Hj << h11, h12, float(0.0), float(0.0),
          h21, h22, float(0.0), float(0.0),
          h31, h32, h11, h12;

    return Hj;

}
