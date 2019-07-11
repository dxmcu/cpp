#include "KalmanFilter.h"


CKalmanFilter::CKalmanFilter()
{
	InitParameters(0.1);
	error_u = 0.0;
	mat_r_lidar << 0.00025, 0,
		0, 0.00025;

	mat_r_radar << 0.09, 0.0, 0.0,
		0.0, 0.09, 0.0,
		0.0, 0.0, 0.09;



	iter_mat_p << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	mat_h << 1, 0, 0, 0,
		0, 1, 0, 0;
}


CKalmanFilter::~CKalmanFilter()
{
}
void CKalmanFilter::DoFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x, EmSensorType sensor_type, Eigen::Vector4d* updated_filterd_x , float delta_time)
{
	if (sensor_type == EmSensorType::SENSOR_LIDAR)
	{
		DoLidarFilter(latest_filtered_x, measured_x, updated_filterd_x,delta_time);
	}
	if (sensor_type == EmSensorType::SENSOR_RADAR)
	{
		DoRadarFilter(latest_filtered_x, measured_x, updated_filterd_x);
	}
}
void CKalmanFilter::DoRadarFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x, Eigen::Vector4d* updated_filterd_x)
{
	double px =  (*measured_x)(0);
	double py =  (*measured_x)(1);
	double vx = (*measured_x)(2);
	double vy = (*measured_x)(3);

	double c1 = px*px+ py*py;
	double c2 = sqrt(c1);
	double c3 = c1 * c2;

	Eigen::Matrix<double, 3, 4> H_Jac;
	if (c1 < 1e-08 || c2 < 1e-08|| c3 < 1e-08)
		H_Jac << 0, 0, 0, 0,
				 0, 0, 0, 0,
				 0, 0, 0, 0;
	else
		H_Jac << px / c2, py / c2, 0, 0,
				-py / c1, px / c1, 0, 0,
				py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
	double angle = 0.0;
	if (px > 1e-08)
		angle = atan(py / px);
	double ro = c2;
	double ros = (vx*px + vy*py) / c2;

	Eigen::Vector3d  z;
	z << ro, angle, ros;

	Eigen::Vector3d y = z - (H_Jac * (*measured_x));
	Eigen::Matrix<double, 3, 3> s  = H_Jac * iter_mat_p * H_Jac.transpose() + mat_r_radar;
	Eigen::Matrix<double, 4, 3> k = iter_mat_p * H_Jac.transpose() * s.inverse();
	*updated_filterd_x  = *measured_x + (k * y);

	iter_mat_p = (Eigen::MatrixXd::Identity(4, 4) - (k * H_Jac)) * iter_mat_p;

}
void CKalmanFilter::DoLidarFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x,Eigen::Vector4d* updated_filterd_x , float delta_time)
{
	InitParameters(delta_time);
	Eigen::Vector4d x = (mat_f*(*latest_filtered_x)) + mat_b.transpose()*error_u;
	Eigen::Matrix<double, 4, 4> p = mat_f * iter_mat_p * mat_f.transpose() + mat_q;
	Eigen::Vector2d z;
	z << (*measured_x)(0), (*measured_x)(1);
	Eigen::Vector2d y = z - (mat_h * x);
	Eigen::Matrix<double, 2, 2> s = mat_h * p * mat_h.transpose() + mat_r_lidar;
	Eigen::Matrix<double, 4, 2> k = p * mat_h.transpose() * s.inverse();
	*updated_filterd_x = x + (k * y);
	iter_mat_p = (Eigen::MatrixXd::Identity(4, 4) - (k * mat_h)) * p;
}
void CKalmanFilter::InitParameters(double delta_time)
{

	double dt2 = delta_time*delta_time ;
	double dt3 = dt2*delta_time;
	double dt4 = dt3*delta_time;

	mat_f << 1, 0, delta_time, 0,
				0, 1, 0, delta_time,
				0, 0, 1, 0,
				0, 0, 0, 1;
	mat_b << dt2/2, dt2/2, delta_time, delta_time;

	mat_q << dt4 / 4, 0.0, dt3 / 2, 0.0,
		0.0, dt4 / 4, 0.0, dt3 / 2,
		dt3 / 2, 0.0, dt2, 0.0,
		0.0, dt3 / 2, 0.0, dt2;
}
