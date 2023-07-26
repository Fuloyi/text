#include <iostream>
#include"kalman_filter.hpp"

using namespace std;
int main() {
	double x = 0.0, y = 0.0;
	double last_timestamp = 0.0, now_timestamp = 0.0;
	KalmanFilter kf;
	while (&x,&y,&now_timestamp)
	{
		if (!kf.IsInitialized())
		{
			last_timestamp = now_timestamp;
			VectorXd x_in(4, 1);
			x_in << x, y, 0.0, 0.0;
			kf.Initialization(x_in);
			MatrixXd P_in(4, 4);
			P_in << 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0;
			kf.Set_P(P_in);
			MatrixXd Q_in(4, 4);
			Q_in<< 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0;
			kf.Set_Q(Q_in);
			MatrixXd H_in(2, 4);
			Q_in << 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0;
			kf.Set_H(H_in);
			MatrixXd R_in(2, 2);
			R_in << 1.0, 0.0,
				0.0, 1.0;
			kf.Set_R(R_in);
		}
		double delta_t = now_timestamp - last_timestamp;
		last_timestamp = now_timestamp;
		MatrixXd F_in(4, 4);
		F_in << 1.0, 0.0, delta_t, 0.0,
			0.0, 1.0, 0.0, delta_t,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;
		kf.Set_F(F_in);
		kf.Prediction();
		VectorXd z(2, 1);
		z << x, y;
		kf.MeasurementUpdate(z);
		VectorXd x_out = kf.Get_X();
		cout << "y:" << x_out(1) << endl;
	}
}
