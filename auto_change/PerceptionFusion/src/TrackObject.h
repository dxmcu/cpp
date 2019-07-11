#pragma once
#include "common_defs.h"
#include "Geometry.h"
class CTrackObject
{
public:
	CTrackObject();
	~CTrackObject();
	void Init();
	void PredictState(unsigned long long  timestamp);
	DeepBlue::Extent2D GetObjectEnvlope();
public:

	int              track_id;    //目标跟踪的唯一ID
	const  float     close_limit =  1.0;
	//跟踪时间相关参数
	unsigned long long         start_time;
	unsigned long long         latest_time;
	unsigned long long         duation;

	
	//最近融合的位置和速度

	Eigen::Vector3d latest_anchor_point;
	Eigen::Vector3d latest_size;
	Eigen::Vector3d latest_velocity;
	Eigen::Vector3d latest_direction;
	Eigen::Vector3d latest_corners[4];
	Eigen::Vector3d latest_vehicle_pos;
	float           latest_vehicle_angle;
	// buffer for predict data
	mutable MlfPredict predict_;

	// ***************************************************
	// measurement correlative infomation 
	// ***************************************************
	StrTargetStatus  target_status;


	bool is_background = false;

	bool Close2Update(StrTargetStatus& target_status , Eigen::Vector3d vehicle_pos , float angle);

};


typedef std::shared_ptr<CTrackObject> TrackedObjectPtr;