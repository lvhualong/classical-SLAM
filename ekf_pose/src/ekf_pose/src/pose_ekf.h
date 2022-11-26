#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

class Pose_ekf
{
public:
	Pose_ekf();
	~Pose_ekf();	
	void predict(Vector3d gyro, Vector3d acc, double t);
	void correct(Vector3d pos, Vector3d vel, Vector3d mag, double t);
	void process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F, MatrixXd& G);
	MatrixXd computeF(Vector3d gyro, Vector3d acc);
	
	VectorXd measurement(VectorXd x, Vector3d mag);
	MatrixXd computeH(Vector3d mag);

	void measurement_fix(Vector2d& position, MatrixXd &H);
	void measurement_fix_velocity(Vector3d& velocity, MatrixXd& H);
	void measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H);
	void measurement_magnetic_field(Vector3d& magnetic_field, MatrixXd& H);
	void measurement_gravity(Vector3d& acc, MatrixXd& H);

	void correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R);
	void correct_fix(Vector3d position, double t);
	void correct_fix_velocity(Vector3d velocity, double t);
	void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch
	void correct_magnetic_field(Vector3d mag, double t);
	void correct_gravity(Vector3d acc, double t);
	// void measurement_altimeter(double& altimeter_height, MatrixXd H);
	void getState(Quaterniond& q, Vector3d& position, Vector3d& velocity, Vector3d & bw, Vector3d&  ba);
	double get_time() { return current_t;}
private:
	VectorXd x;//state 
	MatrixXd P;//covariance


	const Vector3d GRAVITY = Vector3d(0, 0, 9.8);
	//covariance parameter
	const double fix_cov = 2.0;
	const double sonar_height_cov = 0.2;
	const double fix_velocity_cov = 2.0;
	
	const double gyro_cov = 0.01;
	const double acc_cov = 0.1;

	const double gravity_cov = 5.0;
	const double mag_cov = 5.0;

	const int n_state = 16;
	MatrixXd Q;//imu observation noise
	const MatrixXd R_fix = Matrix2d::Identity()*fix_cov;
	const MatrixXd R_fix_velocity = Matrix3d::Identity()*fix_velocity_cov;
	const MatrixXd R_sonar_height = MatrixXd::Identity(1, 1)*sonar_height_cov;
	const MatrixXd R_magnetic = Matrix3d::Identity()*mag_cov;
	const MatrixXd R_gravity = Matrix3d::Identity()*gravity_cov;

	Vector3d acc;
	Vector3d gyro;

	Vector3d referenceMagneticField_;
	double current_t;
	bool initialized;

	bool fix_initialized;
	bool imu_initialized;
	bool altimeter_initialized;
	bool sonar_initialized;
	bool magnetic_initialized;
	
};

#endif 