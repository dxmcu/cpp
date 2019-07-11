#include "TrackObject.h"
#include "GeoUtil.h"

CTrackObject::CTrackObject()
{
	Init();
}


CTrackObject::~CTrackObject()
{

}

void CTrackObject::Init()
{
	track_id = -1;
	
	start_time = 0;
	latest_time = 0;
	duation = 0;

	latest_anchor_point(0) = 0.0;
	latest_anchor_point(1) = 0.0;
	latest_anchor_point(2) = 0.0;

	latest_size(0) = 0.0;
	latest_size(1) = 0.0;
	latest_size(2) = 0.0;

	latest_velocity(0) = 0.0;
	latest_velocity(1) = 0.0;
	latest_velocity(2) = 0.0;

	for (int i = 0; i < 4; i++)
	{
		latest_corners[i](0) = 0.0;
		latest_corners[i](1) = 0.0;
		latest_corners[i](2) = 0.0;
	}

	latest_vehicle_angle = 0.0;
}
void CTrackObject::PredictState(unsigned long long  timestamp)
{
	double  time_diff = ( timestamp - latest_time )/1000.0;
	predict_.state.resize(18);
	predict_.state(0) = static_cast<float>(latest_anchor_point(0) +
		latest_velocity(0) * time_diff);
	predict_.state(1) = static_cast<float>(latest_anchor_point(1) +
		latest_velocity(1) * time_diff);
	predict_.state(2) = static_cast<float>(latest_anchor_point(2) +
		latest_velocity(2) * time_diff);
	predict_.state(3) = static_cast<float>(latest_velocity(0));
	predict_.state(4) = static_cast<float>(latest_velocity(1));
	predict_.state(5) = static_cast<float>(latest_velocity(2));

	for (int i = 0; i < 4; i++)
	{
		predict_.state(3 * (2 + i) + 0) = static_cast<float>(latest_corners[i](0) +
			latest_velocity(0) * time_diff);
		predict_.state(3 * (2 + i) + 1) = static_cast<float>(latest_corners[i](1) +
			latest_velocity(0) * time_diff);
		predict_.state(3 * (2 + i) + 2) = static_cast<float>(latest_corners[i](1) +
			latest_velocity(0) * time_diff);
	}

}
DeepBlue::Extent2D CTrackObject::GetObjectEnvlope()
{
	DeepBlue::Extent2D extent;

	extent.min_x = latest_anchor_point(0) - target_status.fLength / 2.0;
	extent.max_x = latest_anchor_point(0) + target_status.fLength / 2.0;
	extent.max_y = latest_anchor_point(1) + target_status.fWidth / 2.0;
	extent.min_y = latest_anchor_point(1) - target_status.fWidth / 2.0;


	//以下代码留给激光雷达处理
	//for (int i = 0; i < 4; i++)
	//{
	//	if (extent.min_x > latest_corners[i](0))
	//		extent.min_x = latest_corners[i](0);
	//	if (extent.max_x < latest_corners[i](0))
	//		extent.max_x = latest_corners[i](0);

	//	if (extent.min_y > latest_corners[i](1))
	//		extent.min_y = latest_corners[i](1);
	//	if (extent.max_y < latest_corners[i](1))
	//		extent.max_y = latest_corners[i](1);
	//}

	return extent;

}

bool CTrackObject::Close2Update(StrTargetStatus& update_target_status , Eigen::Vector3d vehicle_pos , float angle)
{
	Eigen::Vector3d  track_pos;
	Eigen::Vector3d  update_pos;
	track_pos << target_status.fX, target_status.fY, 0.0;
	update_pos << update_target_status.fX, update_target_status.fY, 0.0;

	CGeoUtil::Vehicle2Enu(track_pos,latest_vehicle_pos , latest_vehicle_angle , track_pos);
	CGeoUtil::Vehicle2Enu(update_pos, vehicle_pos, angle, update_pos);

	double lx1  = track_pos(0) - target_status.fWidth * 0.5;
	double rx1 = track_pos(0) + target_status.fWidth * 0.5;
	double uy1 = track_pos(1) + target_status.fLength * 0.5;
	double dy1 = track_pos(1)  - target_status.fLength * 0.5;

	double lx2 = update_pos(0) - update_target_status.fWidth * 0.5;
	double rx2 = update_pos(0) + update_target_status.fWidth * 0.5;
	double uy2 = update_pos(1) + update_target_status.fLength * 0.5;
	double dy2 = update_pos(1) - update_target_status.fLength * 0.5;



	if (lx2 > rx1 || rx2 < lx1 || uy2<dy1 || dy2>uy1)
		return false;
	
	//如果包含在跟踪目标范围之内，则直接返回也不再新增
	if (lx2 > lx1 && lx2 < rx1 && rx2 >lx1 && rx2 < rx1 && uy2 < uy1 && uy2 > dy1 && dy2 < uy1 && dy2 > dy1)
		return true;

	double dx = std::fabs(target_status.fX - update_target_status.fX);
	double dy = std::fabs(target_status.fY - update_target_status.fY);

	double new_width = target_status.fWidth + update_target_status.fWidth - 2 * dy;
	double new_length = target_status.fLength + update_target_status.fLength - 2 * dx;
	
	target_status.fX = (target_status.fX + update_target_status.fX) / 2.0;
	target_status.fY = (target_status.fY + update_target_status.fY) / 2.0;

	target_status.fWidth = new_width;
	target_status.fLength = new_length;

	latest_anchor_point(0) = target_status.fX;
	latest_anchor_point(0) = target_status.fY;

	return true;
}
