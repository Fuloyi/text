#include"kalman_filter.hpp"
	KalmanFilter::KalmanFilter()
	{
		is_initialized_ = false;
	};
	VectorXd KalmanFilter::Get_X()
	{
		return x_;
	}
	bool KalmanFilter::IsInitialized()
	{
		return is_initialized_;
	}
	void KalmanFilter::Initialization(VectorXd x_in)
	{
		x_ = x_in;
		is_initialized_ = true;
	}
	void KalmanFilter::Set_F(VectorXd F_in)
	{
		F_ = F_in;
	}
	void KalmanFilter::Set_P(VectorXd P_in)
	{
		P_ = P_in;
	}
	void KalmanFilter::Set_Q(VectorXd Q_in)
	{
		Q_ = Q_in;
	}
	void KalmanFilter::Set_H(VectorXd H_in)
	{
		H_ = H_in;
	}
	void KalmanFilter::Set_R(VectorXd R_in)
	{
		R_ = R_in;
	}

	void KalmanFilter::Prediction() {
		x_ = F_ * x_;
		MatrixXd Ft = F_.transpose();
		P_ = F_ * P_ * Ft + Q_;
	}
	void KalmanFilter::MeasurementUpdate(const VectorXd& z) {
		VectorXd y = z - H_ * x_;
		MatrixXd S = H_ * P_ * H_.transpose() + R_;
		MatrixXd K = P_ * H_.transpose() * S.inverse();
		x_ = x_ + (K * y);
		int size = x_.size();
		MatrixXd I = MatrixXd::Identity(size, size);
		P_ = (I - K * H_) * P_;
	}