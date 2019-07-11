#include "GeoUtil.h"


CGeoUtil::CGeoUtil()
{
}


CGeoUtil::~CGeoUtil()
{
}
void CGeoUtil::Vehicle2Enu(Eigen::Vector3d track_pos, Eigen::Vector3d v_pos, float angle, Eigen::Vector3d& enu_point)
{
	double _angle = angle*PI / 180.0;
	float x = cos(_angle)*track_pos(0) - sin(_angle)*track_pos(1);
	float y = sin(_angle)*track_pos(0) + cos(_angle)*track_pos(1);
	x += v_pos(0);
	y += v_pos(1);
	enu_point << x, y, 0.0;
}

void CGeoUtil::Vehicle2Enu(Point o_pos , Point v_pos , float angle , Point& enu_point)
{
	double _angle = angle*PI / 180.0;
	enu_point.X = cos(_angle)*o_pos.X - sin(_angle)*o_pos.Y;
	enu_point.Y = sin(_angle)*o_pos.X + cos(_angle)*o_pos.Y;
	enu_point.Z = 0.0;
	enu_point.X += v_pos.X;
	enu_point.Y += v_pos.Y;
}
void CGeoUtil::Enu2Vehicle(Point o_enu_pos, Point v_pos, float angle, Point& o_pos)
{
	double _angle = angle*PI / 180.0;
	o_enu_pos.X -= v_pos.X;
	o_enu_pos.Y -= v_pos.Y;
	o_pos.X = cos(-_angle)*o_enu_pos.X - sin(-_angle)*o_enu_pos.Y;
	o_pos.Y = sin(-_angle)*o_enu_pos.X + cos(-_angle)*o_enu_pos.Y;
	o_pos.Z = 0.0;

}
//经纬高转换到地球坐标系
void CGeoUtil::LBH2Wgs84(MapPoint point, Point *pWGS84Point)
{
	double fmajor_axis = 6378137;
	double fminor_axis = 6356752.3;
	double feccentricity_square = (fmajor_axis * fmajor_axis - fminor_axis * fminor_axis) / (fmajor_axis * fmajor_axis);
	double flatitude = point.Lat * PI / 180;
	double flongitude = point.Lon * PI / 180;
	double fAltitude = point.Alt;
	double radius_earth = fmajor_axis / sqrt(1 - feccentricity_square * sin(flatitude) * sin(flatitude));
	//printf("flatitude = %lf, flongitude = %lf,fAltitude = %lf,radius_earth = %lf\n", flatitude, flongitude, fAltitude, radius_earth);

	pWGS84Point->X = (radius_earth + fAltitude)*cos(flatitude) * cos(flongitude);
	pWGS84Point->Y = (radius_earth + fAltitude)*cos(flatitude) * sin(flongitude);
	pWGS84Point->Z = (radius_earth*(1 - feccentricity_square) + fAltitude) * sin(flatitude);
	//printf("X:%f Y:%f Z:%f\n", pWGS84Point->X, pWGS84Point->Y, pWGS84Point->Z);
}

//两个地理坐标系转ENU坐标系
void CGeoUtil::WGS842Enu(MapPoint OriginPoint, MapPoint TargetPoint, Point *EnuPoint)
{
	Point temppoint_origin;
	Point temppoint;
	Point temp;

	LBH2Wgs84(OriginPoint, &temppoint_origin);
	LBH2Wgs84(TargetPoint, &temppoint);

	double fLatitude = OriginPoint.Lat * PI / 180;
	double fLongitude = OriginPoint.Lon * PI / 180;
	double fAltitude = OriginPoint.Alt;

	temp.X = temppoint.X - temppoint_origin.X;
	temp.Y = temppoint.Y - temppoint_origin.Y;
	temp.Z = temppoint.Z - temppoint_origin.Z;

	EnuPoint->X = -sin(fLongitude) * temp.X + cos(fLongitude) * temp.Y;
	EnuPoint->Y = -sin(fLatitude) * cos(fLongitude) * temp.X - sin(fLatitude) * sin(fLongitude) * temp.Y + cos(fLatitude) * temp.Z;
	EnuPoint->Z = cos(fLongitude) * cos(fLatitude)* temp.X + cos(fLatitude) * sin(fLongitude) * temp.Y + sin(fLatitude) * temp.Z;
	//printf("X = %f Y = %f Z= %f\n", EnuPoint->X, EnuPoint->Y, EnuPoint->Z);
}
