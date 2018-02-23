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
	 * Calculate the RMSE here.
	 */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here

	if (estimations.size() ==0 || ground_truth.size()== 0 || estimations.size() != ground_truth.size() ) {
		cout <<"one of the estimation or ground truth is either 0 or their size not equal";
		return rmse;
	}


	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		// estimations[i] and ground_truth[i] is  vector of size 4 containg,px,py,vx,vy
		//difference of each state element is taken then squared and added to cumulative sum of rmse
		VectorXd diff=estimations[i]-ground_truth[i];
		diff=diff.array()*diff.array();
		rmse +=diff;

	}

	//calculate the mean
	rmse=rmse/estimations.size();

	//calculate the squared root

	rmse=rmse.array().sqrt();
	//return the result
	//record rmse
	//cout << "RMSE: "<<rmse<<endl;

	return rmse;
}
