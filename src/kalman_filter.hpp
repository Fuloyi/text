//#ifndef KAERMAN_H_
//#define KAERMAN_H_

#include<Eigen/Dense>
#include<iostream>
using namespace Eigen;
using namespace std;

class KalmanFilter {
private:
	bool is_initialized_;
	VectorXd x_;
	MatrixXd F_;
	MatrixXd P_;
	MatrixXd Q_;
	MatrixXd H_;
	MatrixXd R_;
public:
	KalmanFilter();
	//~KalmanFilter();
	VectorXd Get_X();
	bool IsInitialized();
	void Initialization(VectorXd x_in);
	void Set_F(VectorXd F_in);
	void Set_P(VectorXd P_in);
	void Set_Q(VectorXd Q_in);
	void Set_H(VectorXd H_in);
	void Set_R(VectorXd R_in);
	void Prediction();
	void MeasurementUpdate(const VectorXd& z);
};