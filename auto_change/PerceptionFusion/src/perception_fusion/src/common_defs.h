#pragma once

#include "Eigen/Core"
#include "data_struct_define.h"

struct MlfPredict {
	Eigen::VectorXf state;
	double timestamp;

	void Reset() {
		state.setZero();
		timestamp = 0.0;
	}
};

struct UpdateTargetData
{
	StrTargetFusion target_data;
	int             total_count;
	int             lidar_count;
	int             radar_count;
	int             camera_count;
};

//distance loss type
typedef enum _Enum_DISTANCE_LOSS_TYPE
{

	Center = 0x01,            // 中心点距离
	Corners = 0x02,           // 角点距离
	Iou = 0x04,               // IOU距离 
	Velocity = 0x08           // 速度距离

}Enum_DISTANCE_LOSS_TYPE;
