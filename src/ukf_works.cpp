#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// State dimension
	n_x_=5;

	// initial state vector
	x_ = VectorXd(n_x_);
	x_.fill(0);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);
	P_.fill(0);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 30;

	//DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;
	//DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

	//ukf object initialization state, initially set to false, set to true in first call of ProcessMeasurement
	is_initialized_=false;



	//Augmented state dimension
	n_aug_=n_x_+2;

	//sigma point length
	n_sig_=2*n_aug_+1;

	//predicted sigma points matrix
	Xsig_pred_=MatrixXd(n_x_,n_sig_);
	Xsig_pred_.fill(0);
	//time when the state is true, in us
	time_us_=0;


	//Sigma point spreading parameter
	lambda_=3-n_aug_;


	///* Weights of sigma points
	weights_=VectorXd(n_sig_);
	//value of weights_ is static, genece init in constructor
	//weights_.fill(0.0);
	weights_.fill(1/(2*(lambda_+n_aug_)));
	weights_(0)=lambda_/(lambda_+n_aug_);

	NIS_=0;




}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	/*****************************************************************************
	 *   Initialization
	 *****************************************************************************/
	if (!is_initialized_) {
		/**
		 * Initialize the state x_ with the first measurement.
		 * Create the covariance matrix.
		 * convert radar from polar to cartesian coordinates.
		 */

		// first measurement
		//cout << "UKF: " << endl;
		//if sensor data is from RADAR, measurement data --> rho/phi and rhodot
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			      Convert radar from polar to cartesian coordinates and initialize state.
			 */
			float rho=meas_package.raw_measurements_[0];
			float phi=meas_package.raw_measurements_[1];
			float rhodot=meas_package.raw_measurements_[2];

			//convert polar to cartesian cords
			//rho,phi is  specified anticlockwise from x (considered vertical) and y considered
			//horiz pointing to left
			float px= rho*cos(phi);
			float py=rho*sin(phi);
			float vx=rhodot*cos(phi);
			float vy=rhodot*sin(phi);
			float v=sqrt(vx*vx+vy*vy); // rhodot :))
			cout <<"Radar init, rho,phi,px,py,vx,vy,rhodot,v"<<rho<<phi<<px<<py<<vx<<vy<<v<<rhodot<<endl;
			x_<<px,py,v,0,0;

			P_ <<   1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1, 0, 0,
					0, 0, 0, 1, 0,
					0, 0, 0, 0, 1;
			//cout <<"Radar init, px,py,vx,vy"<<x_ <<endl;
		}
		//if sensor data is from LIDAR, measurement data --> px and py directly
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			      Initialize state.
			 */
			//set the state with the initial location and zero velocity


			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0,0;
			P_ << 1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1, 0, 0,
					0, 0, 0, 1, 0,
					0, 0, 0, 0, 1;

			//cout <<"Lidar init, px,py,vx,vy"<<x_ <<endl;
		}
		//save initial time stamp
		time_us_ = meas_package.timestamp_;
		// done initializing, no need to predict or update

		//set weights_
		/*double weight_0 = lambda_/(lambda_+n_aug_);
		weights_(0) = weight_0;
		for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
			double weight = 0.5/(n_aug_+lambda_);
			weights_(i) = weight;
		} */

		cout <<"Init Done"<<endl;
		is_initialized_ = true;
		return;
	}

	//calculate dt
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
	//cout << "curr time is: "<<meas_package.timestamp_<<endl;
	//cout<<"prev time"<<time_us_<<endl;
	//cout << "DT is: "<<dt<<endl;
	time_us_ = meas_package.timestamp_;

	/***********************************************************
	 * Use the sensor type to perform the Predict/update step.
	 ***********************************************************/

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		cout << "RADAR data is being used "<<endl;
		//predict only if radar data use_radar_ is true
		//cout <<"Pred start radar "<<endl;
		Prediction(dt);
		cout <<"Pred for radar done"<<endl;
		//cout <<"P_"<<P_<<endl;
		//cout<<"x_"<<x_<<endl;

		// Radar updates
		//cout <<"update start radar "<<endl;
		UpdateRadar(meas_package);
		cout <<"update for radar done"<<endl;
		//cout <<"P_"<<P_<<endl;
		//cout<<"x_"<<x_<<endl;
	} else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
		cout << "LASER meas is being used  "<<endl;
		//predict only if laser data use_laser_ is true
		//cout <<"Pred start laser "<<endl;
		Prediction(dt);
		//cout <<"Pred for Laser done"<<endl;
		//cout <<"P_"<<P_<<endl;
		//cout<<"x_"<<x_<<endl;
		// Laser updates
		//cout <<"update start laser "<<endl;
		UpdateLidar(meas_package);
		//cout <<"update for laser done"<<endl;
		//cout <<"P_"<<P_<<endl;
		//cout<<"x_"<<x_<<endl;

	}

	//// print the output
	//cout << "x_ = " << ekf_.x_ << endl;
	//cout << "P_ = " << ekf_.P_ << endl;




}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**

  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
	 */

	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

	/*******************************************************************************
	 * Augmented sigma point generation
	 ******************************************************************************/

	//create augmented mean state
	x_aug.head(n_x_) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_,n_x_) = P_;
	P_aug(5,5) = std_a_*std_a_;
	P_aug(6,6) = std_yawdd_*std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0)  = x_aug;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
	}

	/****************************************************
	 *Sigma point prediction
	 ***************************************************/
	//predict sigma points
	for (int i = 0; i< n_sig_; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);
		double nu_yawdd = Xsig_aug(6,i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;
	}

	/*// set weights
	double weight_0 = lambda/(lambda+n_aug);
	weights(0) = weight_0;
	for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
		double weight = 0.5/(n_aug+lambda);
		weights(i) = weight;
	}
	std::cout<<"Weights"<<weights<<std::endl; */

	//predicted state mean
	x_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
		x_ = x_+ weights_(i) * Xsig_pred_.col(i);
	}

	//predicted state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
	}


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
	 */

	//calc predicted state, H is measurement function which is linear for linear kalman function
	MatrixXd H_laser =MatrixXd(2,n_x_);
	H_laser << 1,0,0,0,0,
			0,1,0,0,0;
	VectorXd z_pred = H_laser * x_;
	VectorXd z=meas_package.raw_measurements_;
	//y is difference between predicted and measured state
	VectorXd y = z - z_pred;


	MatrixXd R_laser=MatrixXd(2,2);
	R_laser <<    std_laspx_*std_laspx_, 0,
			0,std_laspy_*std_laspy_;
	//aply kalman filter equations and update state and covariance mtx with kalman gain,K
	//the statements below are common to linear and extended kalman, can be created in a function,
	//function stack vs inline for processing speed....
	MatrixXd Ht = H_laser.transpose();
	MatrixXd S = H_laser * P_ * Ht + R_laser;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser) * P_;

	cout<<"Updated x_ LASER: "<<x_<<endl;
	cout<<"Updated P_ LASER:  "<<P_<<endl;


	double NIS_laser = y.transpose() * S.inverse() * y;
	cout<<"NIS_laser: "<<NIS_laser<<endl;
	NIS_=NIS_laser;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
 	 Use radar data to update the belief about the object's
  	  position. Modify the state vector, x_, and covariance, P_.

  	  You'll also need to calculate the radar NIS.
	 */
	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;
	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, n_sig_);

	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0,i);
		double p_y = Xsig_pred_(1,i);
		double v  = Xsig_pred_(2,i);
		double yaw = Xsig_pred_(3,i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
		Zsig(1,i) = atan2(p_y,p_x);                                 //phi
		Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i=0; i < n_sig_; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z,n_z);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z,n_z);
	R <<    std_radr_*std_radr_, 0, 0,
			0, std_radphi_*std_radphi_, 0,
			0, 0,std_radrd_*std_radrd_;
	S = S + R;

	VectorXd z = VectorXd(n_z);
	z << meas_package.raw_measurements_;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();

	cout<<"Updated x_ RADAR x: "<<x_<<endl;
	cout<<"Updated P_ RADAR:  "<<P_<<endl;
	double NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
	cout<<"NIS_radar: "<<NIS_radar<<endl;
	NIS_=NIS_radar;


}
