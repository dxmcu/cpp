#pragma once
#include "common_defs.h"
#include "Geometry.h"
class CSensorData
{
public:
	CSensorData();
	~CSensorData();

	DeepBlue::Extent2D GetObjectEnvlope();
public:
	// range from 0 to max_match_distance
	float association_score = 0.0f;

	unsigned long long time_stamp = 0.0f;
	StrTargetStatus  target_status;

	Eigen::Vector3d measured_anchor_point;
	Eigen::Vector3d measured_size;
	Eigen::Vector3d measured_velocity;
	Eigen::Vector3d measured_direction;
	Eigen::Vector3d measured_corners[4];

	Eigen::Vector3d  vehicle_pos;
	float            vehicle_angle;
};

typedef std::shared_ptr<CSensorData> UpdateObjectPtr;