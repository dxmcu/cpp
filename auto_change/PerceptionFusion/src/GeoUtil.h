#pragma once
#include "data_struct_define.h"

class CGeoUtil
{
public:
	CGeoUtil();
	~CGeoUtil();
public:
	static void  Vehicle2Enu(Eigen::Vector3d track_pos, Eigen::Vector3d v_pos, float angle, Eigen::Vector3d& enu_point);
	static void  Enu2Vehicle(Point o_enu_pos, Point v_pos, float angle, Point& o_pos);
	static void  Vehicle2Enu(Point o_pos, Point v_pos, float angle, Point& enu_point);
	static void  LBH2Wgs84(MapPoint point, Point *pWGS84Point);
	static void  WGS842Enu(MapPoint OriginPoint, MapPoint TargetPoint, Point *EnuPoint);
};

