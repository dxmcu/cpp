#pragma once

#include "data_struct_define.h"

class CKalmanFilter
{
public:
  CKalmanFilter();
  ~CKalmanFilter();

  /*
  * \brief 初始化卡尔曼滤波器初始参数
  * param[in] delta_time 初始预估的时间误差
  */
  void InitParameters(double delta_time);

  /*
  * \brief 执行卡尔曼滤波
  * param[in] latest_filtered_x  上一次滤波的结果值
  * param[in] measured_x         本次测量值
  * param[in] sensor_type        传感器类型；
  * param[out] updated_filterd_x 本次滤波的结果值
  */
  void DoFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x, EmSensorType sensor_type, Eigen::Vector4d* updated_filterd_x , float delta_time=0.0);

  /*
  * \brief 针对激光雷达执行卡尔曼滤波
  * param[in] latest_filtered_x  上一次滤波的结果值
  * param[in] measured_x         本次测量值
  * param[in] sensor_type        传感器类型；
  * param[out] updated_filterd_x 本次滤波的结果值
  */
  void DoLidarFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x, Eigen::Vector4d* updated_filterd_x , float delta_time);

  /*
  * \brief 针对激光雷达执行扩展卡尔曼滤波
  * param[in] latest_filtered_x  上一次滤波的结果值
  * param[in] measured_x         本次测量值
  * param[in] sensor_type        传感器类型；
  * param[out] updated_filterd_x 本次滤波的结果值
  */
  void DoRadarFilter(Eigen::Vector4d* latest_filtered_x, Eigen::Vector4d* measured_x, Eigen::Vector4d* updated_filterd_x);
public:
  Eigen::Matrix<double, 4, 4> iter_mat_p;       //迭代的协方差矩阵

  Eigen::Matrix<double, 4, 4> mat_f;       //物理模型
  Eigen::Matrix<double, 1, 4> mat_b;       //物理模型误差
  Eigen::Matrix<double, 2, 2> mat_r_lidar; //激光雷达误差矩阵
  Eigen::Matrix<double, 3, 3> mat_r_radar; //毫米波雷达误差矩阵
  Eigen::Matrix<double, 2, 4> mat_h;       //单位换算矩阵
  Eigen::Matrix<double, 4, 4> mat_q;       //测量误差矩阵

  float error_u;
};
