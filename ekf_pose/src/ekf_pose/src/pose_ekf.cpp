#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>
#include "conversion.h"
using namespace std;
using namespace Eigen;


//quaternion: body fram to navigation frame
//Rnb
// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

Matrix3d skew_symmetric(Vector3d v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
		 v(2), 0,  -v(0),
		 -v(1), v(0), 0;
	return m;
}

//diff_(p*q) /diff_q
Matrix4d diff_pq_q(Quaterniond p)
{
	double p0 = p.w();
	Vector3d pv = p.vec();

	Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}


//diff_(p*q)/ diff_p
Matrix4d diff_pq_p(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

//diff_(q*v*q_star)/ diff_q
MatrixXd diff_qvqstar_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v + skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() - q0*skew_symmetric(v));
	return D; 
}

//diff_(qstar*v*q)/ diff_q
MatrixXd diff_qstarvq_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v - skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() + q0*skew_symmetric(v));
	return D; 
}
//diff_(q*v*q_star)/ diff_v
Matrix3d diff_qvqstar_v(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix3d D;
	D = (q0*q0 - qv.dot(qv))*Matrix3d::Identity() + 2*qv*qv.transpose() + 2*q0*skew_symmetric(qv);
	return D; 
}

Pose_ekf::Pose_ekf()
{	
	initialized = false;
	x = VectorXd::Zero(n_state);
	x.head(4) << 1, 0, 0, 0;
	P = MatrixXd::Identity(n_state, n_state);

	Q = MatrixXd::Zero(6, 6);
	Q.block<3, 3>(0, 0) = Matrix3d::Identity()*gyro_cov;
	Q.block<3, 3>(3, 3) = Matrix3d::Identity()*acc_cov;


	initialized = false;
	fix_initialized = false;
	imu_initialized = false;
	altimeter_initialized = false;
	sonar_initialized = false;
	magnetic_initialized = false;
}

Pose_ekf::~Pose_ekf()
{
	
}


void Pose_ekf::predict(Vector3d gyro, Vector3d acc, double t)
{
	if(!imu_initialized)
	{
		imu_initialized = true; initialized = true;
		this->current_t = t;
		double phy = atan2(acc(1), acc(2));
		double theta = atan2(-acc(0), acc(2));
		Vector3d rpy(phy, theta, 0);
		Quaterniond q = euler2quaternion(rpy);
		x(0) = q.w(); x.segment<3>(1) = q.vec();
		return;
	}
	if(t <= current_t) return;

	double dt = t - current_t;
	VectorXd xdot(n_state);
	MatrixXd F(n_state, n_state);
	MatrixXd G(n_state, 6);//G = dx/du

	process(gyro, acc, xdot, F, G);
	
	x += xdot*dt;
	F = MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F
	G = G*dt;
	// cout << "G: " << G << endl;
	// cout << "GQG: " << G*Q*G << endl;

	P = F*P*F.transpose() + G*Q*G.transpose();
	x.head(4).normalize();
	
	this->current_t = t;
	this->acc = acc;
	this->gyro = gyro;
}

//xdot = f(x, u);
void Pose_ekf::process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F, MatrixXd& G)
{
	
	Quaterniond q;
	Vector3d p, v, bw, ba;
	getState(q, p, v, bw, ba);
	
	xdot.setZero();
	F.setZero();
	G.setZero();
	
	Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = gyro - bw;
	Quaterniond q_dot = q*gyro_q;
	q_dot.w() /= 2; q_dot.vec() /= 2;//* and / to scalar is not support for Quaternion
	xdot(0) = q_dot.w(); xdot.segment<3>(1) = q_dot.vec();
	xdot.segment<3>(4) = v;

	Quaterniond acc_b_q(0, 0, 0, 0);
	acc_b_q.vec() = acc - ba;
	Quaterniond acc_n_q =  q*acc_b_q*q.inverse();
	xdot.segment<3>(7) = acc_n_q.vec() - GRAVITY;//body frame to n frame 
	
	F.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
	F.block<4, 3>(0, 10) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));
	F.block<3, 3>(4, 7) = Matrix3d::Identity();
	F.block<3, 4>(7, 0) = diff_qvqstar_q(q, acc_b_q.vec());
	F.block<3, 3>(7, 13) = -diff_qvqstar_v(q);

	//G = d_xdot/du
	G.block<4, 3>(0, 0) = 0.5*diff_pq_q(q).block<4, 3>(0, 1);//diff(0.5*q*gyro_q)/diff(gyro_q)
	G.block<3, 3>(7, 3) = diff_qvqstar_v(q);//diff(q*a*qstar)/diff(a)
}

void Pose_ekf::getState(Quaterniond& q, Vector3d& p, Vector3d& v, Vector3d & bw, Vector3d& ba)
{
	q.w() = x(0);
	q.vec() = x.segment<3>(1);
	p = x.segment<3>(4);
	v = x.segment<3>(7);
	bw = x.segment<3>(10);
	ba = x.segment<3>(13);
}


void Pose_ekf::measurement_fix(Vector2d& position, MatrixXd &H)
{
	position = x.segment<2>(4);
	H = MatrixXd::Zero(2, n_state);
	H.block<2, 2>(0, 4) = Matrix2d::Identity();
}
void Pose_ekf::measurement_fix_velocity(Vector3d& velocity, MatrixXd& H)
{
	velocity = x.segment<3>(7);
	H = MatrixXd::Zero(3, n_state);
	H.block<3, 3>(0, 7) = Matrix3d::Identity();
}

void Pose_ekf::measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H)
{
	sonar_height = VectorXd(1);
	sonar_height(0) = x(6);
	H = MatrixXd::Zero(1, n_state);
	H(0, 6) = 1;
}

void Pose_ekf::measurement_magnetic_field(Vector3d& magnetic_field, MatrixXd& H)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Quaterniond ref_mag_q;
	ref_mag_q.w() = 0; ref_mag_q.vec() = referenceMagneticField_;
	Quaterniond magnetic_field_q =  q.inverse()*ref_mag_q*q; //r_n to r_b
	magnetic_field = magnetic_field_q.vec();

	H = MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, referenceMagneticField_);
}

void Pose_ekf::measurement_gravity(Vector3d& acc, MatrixXd& H)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Vector3d ba = x.segment<3>(13);
	Quaterniond g_n_q;
	g_n_q.w() = 0; g_n_q.vec() = Vector3d(0, 0, 1);//only direction is used
	Quaterniond acc_q =  q.inverse()*g_n_q*q; //r_n to r_b
	acc = acc_q.vec();

	H = MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, GRAVITY);
}


void Pose_ekf::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R)
{
   	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    x += K*(z - zhat);
    
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
    x.head(4).normalize();	
}

void Pose_ekf::correct_fix(Vector3d position, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;
	Vector2d z = position.head(2);
	Vector2d zhat;
	MatrixXd H;
	measurement_fix(zhat, H);
	correct(z, zhat, H, R_fix);
}
void Pose_ekf::correct_fix_velocity(Vector3d velocity, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
	predict(this->gyro, this->acc, t);
	
	Vector3d z = velocity;
	Vector3d zhat;
	MatrixXd H;
	measurement_fix_velocity(zhat, H);
	correct(z, zhat, H, R_fix_velocity);
}
void Pose_ekf::correct_sonar_height(double sonar_height, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;	
	predict(this->gyro, this->acc, t);

	VectorXd z(1);
	z(0) = sonar_height;
	VectorXd zhat(1);
	MatrixXd H;

	measurement_sonar_height(zhat, H);
	correct(z, zhat, H, R_sonar_height);
}

void Pose_ekf::correct_magnetic_field(Vector3d mag, double t)
{
	if(!magnetic_initialized)
	{
		//note, mag in ENU should be [0 1 x], but for the simulated data it is [1 0 x], maybe a bug
		referenceMagneticField_(0) = mag.head(2).norm();//todo
		referenceMagneticField_(1) = 0;
		referenceMagneticField_(2) = mag(2);
		magnetic_initialized = true;
		current_t = t;
		return;
	}
	if(t < current_t) return;
	predict(this->gyro, this->acc, t);
	
	Vector3d z = mag;
	Vector3d zhat;
	MatrixXd H;
	measurement_magnetic_field(zhat, H);
	correct(z, zhat, H, R_magnetic);
}

void Pose_ekf::correct_gravity(Vector3d acc, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	if(t < current_t) return;
	predict(this->gyro, this->acc, t);
	
	Vector3d z = acc/acc.norm();
	Vector3d zhat;
	MatrixXd H;
	measurement_gravity(zhat, H);
	correct(z, zhat, H, R_gravity);
}