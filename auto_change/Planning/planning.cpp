#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <libconfig.h>
#ifndef _UNISTD_H
#define _UNISTD_H
//#include <io.h>
//#include <process.h>
#endif /* _UNISTD_H */

#include <assert.h>
#include <sstream>
#include <iostream> 
#include <zmq.h>
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <map>
#include <string>
#include <iomanip>

#include "data_struct_define.h"
#include "planning.h"
#include "loghelp.h"
#include "route_planner.h"
#include "speed_planner.h"
#include "AlgExcTimer.h"

using namespace std;

map<int, string> Decision_Type_String = { { -1, "Quit" }, { 0, "Forward" }, { 1, "Pass" }, { 2, "LeftPass" }, { 3, "RightPass" }, { 4, "Follow" }, { 5, "Safestop" } };
// 全局变量
StrPlanningConfig strPlanningConfig;
StrControllingConfig strControlConfig;
FILE *pFd = NULL;
FILE *pFd0 = NULL;
FILE *pFd1 = NULL;
StrLocationFusion strCarStatus;
StrTargetFusion strTargetsFusion;
EmDecision UltrasonicDecision = Forward;
EmDecision RadarDecision = Forward;
EmDecision Decision = Forward;			// 每一帧的决策
EmDecision HisDecision = Forward;		// 每一帧决策之后，保存当前帧的决策，用于辅助下一帧的决策
StrSLPoints TargetSLPoints;
int LeftTargetNum = 0;
int RightTargetNum = 0;

float KPURE = 0.0;
StrSLPoints strOutTrajectory;			// 虽然有50个参考点，但是实际的规划轨迹点通常远远小于50，规划的轨迹点数取决于规划终点的S坐标值大小
SLPoint TargetPoint;
SLPoint VehicleSLPoint;
SLPoint DecisionPoint;
// 每一帧规划初始化时，从全局地图pHdMap中直接提取50个点作为每一帧的参考点，保存在ReferencePoint数组中，起始参考点的索引号为iMapId
MapPoint ReferencePoint[50];
// 在每一帧初始化时，StrSLReferLine中存放了每个参考点在车体坐标系下的XYZ坐标、SL坐标系下的SL坐标、theta角度等，iId取值范围是[0,50)
// StrSLReferLine中每个参考点的theta角度表示：车辆在当前参考点的行驶方向与当前帧的车体坐标系X坐标轴之间的夹角，取值一般在[-60,60]区间
StrSLPoints StrSLReferLine;
SLPoint TestSLpoint[3];
int iId = 0;
Global StrReferenceLine;				// 在每一帧初始化时，把50个参考点的XYZ坐标转换到车体坐标系下，保存在StrReferenceLine中
int PlanStatus = 0;
// 保存上一帧规划的轨迹点数据，其中每个点的XYZ坐标是在ENU坐标系下的。如果规划成功，则iNum取值为:3+本次规划点数，否则，iNum取值为：1+本次规划点数
StrSLPoints HisPlanPoints;
SLPoint PlanStartPoint;
int HisId = 0;							// 从规划成功的下一帧开始逐帧更新，保存为上一帧参考点序列HisReferencePoint中离本车位置最近的参考点的索引[0,50]
int iMapId = 0;							// 在每一帧初始化时更新，始终保存着离本车位置最近参考点在地图参考点序列中的索引号
MapPoint HisReferencePoint[50];			// 每一帧规划结束后，保存当前帧的50个参考点数据ReferencePoint到HisReferencePoint中，用于下一帧规划
StrSLPoints HisStrSLReferLine;			// 每一帧规划结束后，保存当前帧的50个参考点数据StrSLReferLine到HisStrSLReferLine中，用于下一帧规划
Point PlanOutStartPoint = { 0, 0.0, 0.0, 0.0 };
Point PlanOutEndPoint = { 0, 0.0, 0.0, 0.0 };
StrHdMap HdMap;							// 全局高精地图，存放在地图HdMap中的参考点的XYZ坐标是在ENU坐标系下的，ENU坐标系的原点是地图HdMap中的起始点
mutex g_mutex_result_update;			// 全局互斥锁：局部路径规划结果更新
condition_variable g_cv_result_update;	// 全局条件变量：局部路径规划结果更新
bool g_flag_result_update = false;		// 全局标志位：局部路径规划结果更新
#if 0
mutex g_mutex_input_request;			// 全局互斥锁：局部路径规划请求输入数据
condition_variable g_cv_input_request;	// 全局条件变量：局部路径规划请求输入数据
bool g_flag_input_request = true;		// 全局标志位：局部路径规划请求输入数据
#endif
vector<thread> threads;
bool debug = false;

double DegreeToRadian(double value)
{
	return value * PI / 180;
}

// 经纬高转换到地球坐标系
void LBH2Wgs84(MapPoint point, Point *pWGS84Point)
{
	double fmajor_axis = 6378137;
	double fminor_axis = 6356752.3;
	double feccentricity_square = (fmajor_axis * fmajor_axis - fminor_axis * fminor_axis) / (fmajor_axis * fmajor_axis);
	double flatitude = point.Lat * PI / 180;
	double flongitude = point.Lon * PI / 180;
	double fAltitude = point.Alt;
	double radius_earth = fmajor_axis / sqrt(1 - feccentricity_square * sin(flatitude) * sin(flatitude));

	pWGS84Point->X = (radius_earth + fAltitude)*cos(flatitude) * cos(flongitude);
	pWGS84Point->Y = (radius_earth + fAltitude)*cos(flatitude) * sin(flongitude);
	pWGS84Point->Z = (radius_earth*(1 - feccentricity_square) + fAltitude) * sin(flatitude);
}

// 两个地理坐标系转ENU坐标系
void WGS842Enu(MapPoint OriginPoint, MapPoint TargetPoint, Point *EnuPoint)
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
}

// 本项目工程中的RFU容易造成误解：虽然这里写的是RFU坐标系，但是实际上用的并不是RFU坐标系，而是ROS系统所使用的车体坐标系
// RFU坐标系：X轴为车的右方向，Y轴为车辆朝前方向，Z轴为车顶向上
// ROS车体坐标系：X轴为车辆超前方向，Y轴为车辆的左方向，Z轴为车顶向上
Point Enu2Rfu(StrLocationFusion *pStrCarStatus, MapPoint *MapGpsPoint)
{
	Point ReturnPoint;

	double deltx = MapGpsPoint->dX - pStrCarStatus->fX;
	double delty = MapGpsPoint->dY - pStrCarStatus->fY;
	double deltz = MapGpsPoint->dZ - pStrCarStatus->fZ;

	Point TempPoint;

	double pitch = DegreeToRadian(pStrCarStatus->pitch);
	double roll = DegreeToRadian(pStrCarStatus->roll);
	double yaw = DegreeToRadian(pStrCarStatus->yaw);

	TempPoint.X = (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)) * deltx
		+ (-cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw)) * delty
		- (sin(roll) * cos(pitch)) * deltz;

	TempPoint.Y = cos(pitch) * sin(yaw) * deltx + cos(pitch) * cos(yaw) * delty
		+ sin(pitch) * deltz;

	TempPoint.Z = (sin(roll) * cos(yaw) - cos(roll) * sin(pitch) * sin(yaw)) * deltx
		+ (-sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw)) * delty
		+ cos(roll) * cos(pitch) * deltz;

	ReturnPoint.X = TempPoint.Y;
	ReturnPoint.Y = -TempPoint.X;
	ReturnPoint.Z = TempPoint.Z;

	return ReturnPoint;
}

// 将本车坐标系下的规划点转到ENU坐标系
Point Rfu2Enu(StrLocationFusion *pStrCarStatus, MapPoint *PlanningPoint)
{
	Point ReturnPoint;
	int i = 0;
	double deltX = PlanningPoint->dX;
	double deltY = PlanningPoint->dY;
	double deltZ = PlanningPoint->dZ;
	double pitch = DegreeToRadian(pStrCarStatus->pitch);
	double roll = DegreeToRadian(pStrCarStatus->roll);
	double yaw = DegreeToRadian(pStrCarStatus->yaw);

	Eigen::MatrixXd Y(3, 1);
	Y(0, 0) = deltX;
	Y(1, 0) = deltY;
	Y(2, 0) = deltY;

	Eigen::MatrixXd R0(3, 3);

	R0 << cos(PI / 2), sin(PI / 2), 0,
		-sin(PI / 2), cos(PI / 2), 0,
		0, 0, 1;

	Eigen::MatrixXd R(3, 3);

	R << (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)), (-cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw)), (-sin(roll) * cos(pitch)),
		(cos(pitch) * sin(yaw)), (cos(pitch) * cos(yaw)), sin(pitch),
		(sin(roll) * cos(yaw) - cos(roll) * sin(pitch) * sin(yaw)), (-sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw)), cos(roll) * cos(pitch);

	Eigen::MatrixXd R2(3, 1);

	R2 = R.transpose() * R0.transpose() * Y;
	ReturnPoint.X = R2(0, 0) + pStrCarStatus->fX;
	ReturnPoint.Y = R2(1, 0) + pStrCarStatus->fY;
	ReturnPoint.Z = R2(2, 0) + pStrCarStatus->fZ;

	return ReturnPoint;
}

// ENU坐标系转本车坐标系
int ConvertVehicleCoordinate(StrLocationFusion *pStrCarStatus, MapPoint *pReferencePoint, Global *pStrReferenceLine)
{
	int i = 0;

	if (pStrReferenceLine->NUM_POINT <= 0)
	{
		printf("Reference Line Num Error\n");
		return -1;
	}

	for (i = 0; i < pStrReferenceLine->NUM_POINT; i++)
	{
		pStrReferenceLine->POINT[i] = Enu2Rfu(pStrCarStatus, &pReferencePoint[i]);
		pStrReferenceLine->POINT[i].ID = i;
	}

	return 0;
}

// 求本车当前位置在地图上的最近的两个临点
int iCorresFinder(StrLocationFusion *pStrCarStatus, StrHdMap *pHdMap)
{
	int iId0 = 0;
	int iId1 = 0;

	int i = 0;
	float fDis = 0.0;
	float fMinDis0 = 0.0;	// 最短距离
	float fMinDis1 = 0.0;	// 次短距离

	double dx = pHdMap->MapPoints[0].dX;
	double dy = pHdMap->MapPoints[0].dY;
	double dz = pHdMap->MapPoints[0].dZ;
	fMinDis0 = sqrt((pStrCarStatus->fX - dx)* (pStrCarStatus->fX - dx) + (pStrCarStatus->fY - dy) * (pStrCarStatus->fY - dy) + (pStrCarStatus->fZ - dz) * (pStrCarStatus->fZ - dz));
	dx = pHdMap->MapPoints[1].dX;
	dy = pHdMap->MapPoints[1].dY;
	dz = pHdMap->MapPoints[1].dZ;
	fMinDis1 = sqrt((pStrCarStatus->fX - dx)* (pStrCarStatus->fX - dx) + (pStrCarStatus->fY - dy) * (pStrCarStatus->fY - dy) + (pStrCarStatus->fZ - dz) * (pStrCarStatus->fZ - dz));
	iId0 = 0;
	iId1 = 1;
	if (fMinDis0 > fMinDis1)
	{
		fDis = fMinDis0;
		fMinDis0 = fMinDis1;
		fMinDis1 = fDis;
		iId0 = 1;
		iId1 = 0;
	}

	for (i = 0; i < pHdMap->iNum; i++)
	{
		dx = pHdMap->MapPoints[i].dX;
		dy = pHdMap->MapPoints[i].dY;
		dz = pHdMap->MapPoints[i].dZ;
		fDis = sqrt((pStrCarStatus->fX - dx)* (pStrCarStatus->fX - dx) + (pStrCarStatus->fY - dy) * (pStrCarStatus->fY - dy) + (pStrCarStatus->fZ - dz) * (pStrCarStatus->fZ - dz));
		if (fDis < fMinDis0)
		{
			fMinDis1 = fMinDis0;
			iId1 = iId0;
			fMinDis0 = fDis;
			iId0 = i;
		}
		else if (fDis > fMinDis0 && fDis < fMinDis1)
		{
			fMinDis1 = fDis;
			iId1 = i;
		}
	}

	if (iId0 == pHdMap->iNum - 1 && iId1 == 0)
		iId = iId0;
	else if (iId1 == pHdMap->iNum - 1 && iId0 == 0)
		iId = iId1;
	else
	{
		if (iId1 > iId0)
			iId = iId0;
		else
			iId = iId1;
	}

	return iId;
}

// 每一帧初始化时，从全局地图pHdMap中直接提取50个点作为每一帧的参考点，保存在ReferencePoint数组中，起始参考点的索引号为iMapId
int iGetReferencePoint(int iId, StrHdMap *pHdMap, MapPoint *pReferencePoint)
{
	int k = 0;
	int i = 0;
	int j = 0;

	if (pHdMap->iNum - iId  < 50)
	{
		for (i = iId; i < pHdMap->iNum; i++)
		{
			pReferencePoint[k].Lat = pHdMap->MapPoints[i].Lat;
			pReferencePoint[k].Lon = pHdMap->MapPoints[i].Lon;
			pReferencePoint[k].Alt = pHdMap->MapPoints[i].Alt;
			pReferencePoint[k].Pitch = pHdMap->MapPoints[i].Pitch;
			pReferencePoint[k].Roll = pHdMap->MapPoints[i].Roll;
			pReferencePoint[k].Yaw = pHdMap->MapPoints[i].Yaw;
			pReferencePoint[k].dX = pHdMap->MapPoints[i].dX;
			pReferencePoint[k].dY = pHdMap->MapPoints[i].dY;
			pReferencePoint[k].dZ = pHdMap->MapPoints[i].dZ;
			k++;
		}

		for (i = 0; i < 50 - pHdMap->iNum + iId; i++)
		{
			pReferencePoint[k].Lat = pHdMap->MapPoints[i].Lat;
			pReferencePoint[k].Lon = pHdMap->MapPoints[i].Lon;
			pReferencePoint[k].Alt = pHdMap->MapPoints[i].Alt;
			pReferencePoint[k].Pitch = pHdMap->MapPoints[i].Pitch;
			pReferencePoint[k].Roll = pHdMap->MapPoints[i].Roll;
			pReferencePoint[k].Yaw = pHdMap->MapPoints[i].Yaw;
			pReferencePoint[k].dX = pHdMap->MapPoints[i].dX;
			pReferencePoint[k].dY = pHdMap->MapPoints[i].dY;
			pReferencePoint[k].dZ = pHdMap->MapPoints[i].dZ;
			k++;
		}
	}
	else
	{
		for (j = iId; j < (iId + 50); j++)
		{
			pReferencePoint[k].Lat = pHdMap->MapPoints[j].Lat;
			pReferencePoint[k].Lon = pHdMap->MapPoints[j].Lon;
			pReferencePoint[k].Alt = pHdMap->MapPoints[j].Alt;
			pReferencePoint[k].Pitch = pHdMap->MapPoints[j].Pitch;
			pReferencePoint[k].Roll = pHdMap->MapPoints[j].Roll;
			pReferencePoint[k].Yaw = pHdMap->MapPoints[j].Yaw;
			pReferencePoint[k].dX = pHdMap->MapPoints[j].dX;
			pReferencePoint[k].dY = pHdMap->MapPoints[j].dY;
			pReferencePoint[k].dZ = 0.0;
			k++;
		}
	}

	return 0;
}

int iMallocMap(StrHdMap *pHdMap)
{

	if (NULL == pHdMap)
	{
		printf("Input Error\n");
		return -1;
	}

	if (pHdMap->iNum <= 0)
	{
		printf("HdMap Num Error\n");
		return -1;
	}

	pHdMap->MapPoints = (MapPoint *)malloc(pHdMap->iNum * sizeof(MapPoint));

	return 0;
}

int iFreeMap(StrHdMap *pHdMap)
{
	if (NULL != pHdMap->MapPoints)
		free(pHdMap->MapPoints);

	return 0;
}

int iReadHdMap(char *pchMapDir, StrHdMap *pHdMap)
{
	int iNum = 0;
	int i = 0;
	int iRet = 0;

	FILE *pFd = NULL;
	pFd = fopen(pchMapDir, "rb");
	if (NULL == pFd)
	{
		printf("Open Map File Failed\n");
		return -1;
	}

	if (!feof(pFd))
	{
		fread(&iNum, sizeof(int), 1, pFd);
		pHdMap->iNum = iNum;
		if (iNum > 0)
		{
			// 分配高精地图内存空间
			iRet = iMallocMap(pHdMap);
			if (0 != iRet)
			{
				printf("Malloc Hd Map Failed\n");
				return -1;
			}
		}
		else
		{
			printf("Hd Map Error\n");
			fclose(pFd);
			pFd = NULL;
			return -1;
		}

		for (i = 0; i < iNum; i++)
		{
			fread(&(pHdMap->MapPoints[i].iId), sizeof(int), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Lat), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Lon), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Alt), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Pitch), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Roll), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].Yaw), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].dX), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].dY), sizeof(double), 1, pFd);
			fread(&(pHdMap->MapPoints[i].dZ), sizeof(double), 1, pFd);
		}
	}

	fclose(pFd);
	pFd = NULL;

	return 0;
}

int iReadPlanningConfig(char *pchDir, StrPlanningConfig *pParamConfig)
{
	config_t strCfg;
	config_setting_t *pSetting;
	config_init(&strCfg);

	// Read the file. If there is an error, report it and exit
	if (!config_read_file(&strCfg, pchDir))
	{
		fprintf(stderr, "%s:%d - %s\n", config_error_file(&strCfg), config_error_line(&strCfg), config_error_text(&strCfg));
		config_destroy(&strCfg);
		return -1;
	}

	pSetting = config_lookup(&strCfg, "planning.param");
	if (pSetting != NULL)
	{
		if (!(config_setting_lookup_float(pSetting, "fSafeDistance1", &pParamConfig->fSafeDis1)
			&& config_setting_lookup_float(pSetting, "fSafeDistance2", &pParamConfig->fSafeDis2)
			&& config_setting_lookup_float(pSetting, "fMaxPlanningDistance", &pParamConfig->fMaxPlanningDistance)
			&& config_setting_lookup_float(pSetting, "max_acc", &pParamConfig->fMaxAcc)
			&& config_setting_lookup_float(pSetting, "max_v", &pParamConfig->fMaxV)
			&& config_setting_lookup_float(pSetting, "follow_dis", &pParamConfig->follow_dis)
			&& config_setting_lookup_float(pSetting, "ultrasonic_speed", &pParamConfig->fUltrasonicSpeed)
			&& config_setting_lookup_float(pSetting, "ultrasonic_dis", &pParamConfig->fUltrasonicDis)
			&& config_setting_lookup_float(pSetting, "fVehicleWidth", &pParamConfig->fVehicleWidth)
			&& config_setting_lookup_float(pSetting, "fVehicleLength", &pParamConfig->fVehicleLength)
			&& config_setting_lookup_float(pSetting, "fSampleBuffer", &pParamConfig->fSampleBuffer)
			&& config_setting_lookup_float(pSetting, "fSampleAccuracy", &pParamConfig->fSampleAccuracy)
			&& config_setting_lookup_float(pSetting, "fSampleTime", &pParamConfig->fSampleTime)
			&& config_setting_lookup_float(pSetting, "fUnitS", &pParamConfig->fUnitS)
			&& config_setting_lookup_float(pSetting, "fl0", &pParamConfig->fl0)
			&& config_setting_lookup_float(pSetting, "fk", &pParamConfig->fk)
			&& config_setting_lookup_float(pSetting, "fb", &pParamConfig->fb)
			&& config_setting_lookup_float(pSetting, "fPathLCost", &pParamConfig->fPathLCost)
			&& config_setting_lookup_float(pSetting, "fPathDLCost", &pParamConfig->fPathDLCost)
			&& config_setting_lookup_float(pSetting, "fPathDDLCost", &pParamConfig->fPathDDLCost)
			&& config_setting_lookup_float(pSetting, "fEndCost", &pParamConfig->fEndCost)
			&& config_setting_lookup_float(pSetting, "fIgnoreL", &pParamConfig->fIgnoreL)
			&& config_setting_lookup_float(pSetting, "fObstacleCollisionCost", &pParamConfig->fObstacleCollisionCost)
			&& config_setting_lookup_float(pSetting, "fObstacleCollisionDis", &pParamConfig->fObstacleCollisionDis)
			&& config_setting_lookup_float(pSetting, "fObstacleBuffer", &pParamConfig->fObstacleBuffer)
			&& config_setting_lookup_float(pSetting, "fLeftHalfRoadWidth", &pParamConfig->fLeftHalfRoadWidth)
			&& config_setting_lookup_float(pSetting, "fRightHalfRoadWidth", &pParamConfig->fRightHalfRoadWidth)
			&& config_setting_lookup_float(pSetting, "dOriginLat", &pParamConfig->dOriginLat)
			&& config_setting_lookup_float(pSetting, "dOriginLon", &pParamConfig->dOriginLon)
			&& config_setting_lookup_float(pSetting, "dOriginAlt", &pParamConfig->dOriginAlt)
			&& config_setting_lookup_int(pSetting, "RecordFileFlag", &pParamConfig->iRecordFile)
			))
		{
			printf("Can not find planning param!\r\n");
			return -1;
		}
		else
			printf("Found planning param!\r\n");
	}

	config_destroy(&strCfg);
	return 0;
}

int iReadControlConfig(char *pchDir, StrControllingConfig *pParamConfig)
{
	config_t strCfg;
	config_setting_t *pSetting = NULL;
	config_init(&strCfg);

	// Read the file. If there is an error, report it and exit
	if (!config_read_file(&strCfg, pchDir))
	{
		fprintf(stderr, "%s:%d - %s\n", config_error_file(&strCfg), config_error_line(&strCfg), config_error_text(&strCfg));
		config_destroy(&strCfg);
		return -1;
	}

	pSetting = config_lookup(&strCfg, "Control.param");
	if (pSetting != NULL)
	{
		if (!((config_setting_lookup_float(pSetting, "ThrottlePushValue_high", &pParamConfig->ThrottlePushValue_high))
			&& (config_setting_lookup_float(pSetting, "ThrottlePushValue_low", &pParamConfig->ThrottlePushValue_low))
			&& (config_setting_lookup_float(pSetting, "Swith_acc", &pParamConfig->Swith_acc))
			&& (config_setting_lookup_float(pSetting, "Kp", &pParamConfig->Kp))
			&& (config_setting_lookup_float(pSetting, "Ki", &pParamConfig->Ki))
			&& (config_setting_lookup_float(pSetting, "Kd", &pParamConfig->Kd))
			&& (config_setting_lookup_float(pSetting, "Brake_speed", &pParamConfig->Brake_speed))
			&& (config_setting_lookup_float(pSetting, "Brake_emergency", &pParamConfig->Brake_emergency))
			&& (config_setting_lookup_float(pSetting, "dt", &pParamConfig->dt))
			&& (config_setting_lookup_int(pSetting, "Enable_Saturationintegrator", &pParamConfig->Enable_Saturationintegrator))
			&& (config_setting_lookup_float(pSetting, "f_saturationintegrator_high", &pParamConfig->f_saturationintegrator_high))
			&& (config_setting_lookup_float(pSetting, "f_saturationintegrator_low", &pParamConfig->f_saturationintegrator_low))
			&& (config_setting_lookup_float(pSetting, "fmaxhingeangle", &pParamConfig->fmaxhingeangle))
			&& (config_setting_lookup_float(pSetting, "ftolerance", &pParamConfig->ftolerance))
			&& (config_setting_lookup_int(pSetting, "max_num_iteration", &pParamConfig->max_num_iteration))
			&& (config_setting_lookup_float(pSetting, "x_polearm", &pParamConfig->x_polearm))
			&& (config_setting_lookup_float(pSetting, "y_polearm", &pParamConfig->y_polearm))
			&& (config_setting_lookup_float(pSetting, "z_polearm", &pParamConfig->z_polearm))
			&& (config_setting_lookup_float(pSetting, "lat_err_upper", &pParamConfig->lat_err_upper))
			&& (config_setting_lookup_float(pSetting, "lat_err_lower", &pParamConfig->lat_err_lower))
			&& (config_setting_lookup_float(pSetting, "lat_speed_err_upper", &pParamConfig->lat_speed_err_upper))
			&& (config_setting_lookup_float(pSetting, "lat_speed_err_lower", &pParamConfig->lat_speed_err_lower))
			&& (config_setting_lookup_float(pSetting, "BrakePushValue", &pParamConfig->BrakePushValue))
			))
		{
			printf("Can not find control param!\r\n");
			return -1;
		}
		else
			printf("Found control param!\r\n");
	}

	config_destroy(&strCfg);
	return 0;
}

int iPlanningInit(char *pchDir, StrPlanningConfig *pStrPlanningConfig)
{
	int iRet = 0;

	if (NULL == pchDir)
	{
		printf("pchDir Error\n");
		return -1;
	}

	if (NULL == pStrPlanningConfig)
	{
		printf("pStrPlanningConfig Error\n");
		return -1;
	}

	iRet = iReadPlanningConfig(pchDir, pStrPlanningConfig);
	if (-1 == iRet)
	{
		printf("Read Planning Config Failed\n");
		return -1;
	}

	return 0;
}

int iReferenceProvider(StrHdMap *pHdMap, StrLocationFusion *pStrCarStatus, StrPlanningConfig *pStrPlanningConfig, Global *pStrReferenceLine)
{
	int i = 0;
	int j = 0;

	double dx = 0.0;
	double dy = 0.0;
	double dz = 0.0;

	if (NULL == pHdMap || NULL == pStrReferenceLine)
	{
		printf("Input Param Error\n");
		return -1;
	}
	// 将本车位置转换到ENU坐标下
	// ENU坐标原点获取
	MapPoint OriginPoint;
	OriginPoint.Lat = pStrPlanningConfig->dOriginLat;
	OriginPoint.Lon = pStrPlanningConfig->dOriginLon;
	OriginPoint.Alt = pStrPlanningConfig->dOriginAlt;
	// 本车当前位置
	MapPoint TargetPoint;
	pStrCarStatus->dAltitude = OriginPoint.Alt;
	pStrCarStatus->pitch = 0.0;
	pStrCarStatus->roll = 0.0;
	TargetPoint.Lat = pStrCarStatus->dLatitude;
	TargetPoint.Lon = pStrCarStatus->dLongitude;
	TargetPoint.Alt = pStrCarStatus->dAltitude;
	// 把本车的经纬高坐标转换成ENU坐标系下的XYZ坐标
	Point EnuPoint;
	WGS842Enu(OriginPoint, TargetPoint, &EnuPoint);
	pStrCarStatus->fX = EnuPoint.X;
	pStrCarStatus->fY = EnuPoint.Y;
	pStrCarStatus->fZ = EnuPoint.Z;
	if (debug)
		printf("ENU坐标原点的纬度:%f,经度:%f,高度:%f \n", pStrPlanningConfig->dOriginLat, pStrPlanningConfig->dOriginLon, pStrPlanningConfig->dOriginAlt);
	if (debug)
		printf("本车当前位置的纬度:%f,经度:%f,高度:%f \n", TargetPoint.Lat, TargetPoint.Lon, TargetPoint.Alt);
	if (debug)
		printf("本车在ENU坐标系下的坐标: X=%f,Y=%f,Z=%f \n", pStrCarStatus->fX, pStrCarStatus->fY, pStrCarStatus->fZ);

	if (0 == PlanStatus)
		// 从地图参考点序列中，查找离本车位置最近的参考点
		iMapId = iCorresFinder(pStrCarStatus, pHdMap);
	else
	{
		// 从上一帧参考点序列HisReferencePoint中，查找离本车位置最近的参考点
		HisId = iFindHisPlanId(pStrCarStatus, HisReferencePoint);
		if (iMapId + HisId > pHdMap->iNum - 1)
			iMapId = iMapId + HisId - pHdMap->iNum;
		else
			iMapId = iMapId + HisId;
	}

	// 每一帧规划初始化时，从全局地图pHdMap中直接提取50个点作为每一帧的参考点，保存在ReferencePoint数组中，起始参考点的索引号为iMapId
	iGetReferencePoint(iMapId, pHdMap, ReferencePoint);
	pStrReferenceLine->NUM_POINT = 50;
	// 在每一帧初始化时，把50个参考点的XYZ转换到车体坐标系下
	ConvertVehicleCoordinate(pStrCarStatus, ReferencePoint, pStrReferenceLine);

	return 0;
}

bool searchReferenceLine(StrLocationFusion *pStrCarStatus, StrHdMap *pHdMap)
{
	int iMapId = iCorresFinder(pStrCarStatus, pHdMap);
	iGetReferencePoint(iMapId, pHdMap, ReferencePoint);
	MapPoint point = ReferencePoint[0];
	double dist = (pStrCarStatus->fX - point.dX) * (pStrCarStatus->fX - point.dX);
	dist += (pStrCarStatus->fY - point.dY) * (pStrCarStatus->fY - point.dY);
	dist += (pStrCarStatus->fZ - point.dZ) * (pStrCarStatus->fZ - point.dZ);
	dist = sqrt(dist);
	// 搜索路段是否成功
	return dist > MAP_MAX_DISTANCE ? false : true;
}

Json::Value getJsonTrajectory(StrSLPoints *strOutTrajectory)
{
	Json::Value jsonTrajectory;
	jsonTrajectory["A0"] = strOutTrajectory->pfC[0];
	jsonTrajectory["A1"] = strOutTrajectory->pfC[1];
	jsonTrajectory["A2"] = strOutTrajectory->pfC[2];
	jsonTrajectory["A3"] = strOutTrajectory->pfC[3];
	jsonTrajectory["SX"] = PlanOutStartPoint.X;
	jsonTrajectory["SY"] = PlanOutStartPoint.Y;
	jsonTrajectory["EX"] = PlanOutEndPoint.X;
	jsonTrajectory["EY"] = PlanOutEndPoint.Y;
	jsonTrajectory["RW"] = 3.5;			// 默认写3.5m，道路宽度
	jsonTrajectory["DE"] = Decision;	// 决策结果
	jsonTrajectory["DDOOR"] = 2000;
	jsonTrajectory["DOOR_FLAG"] = false;
	jsonTrajectory["ERRORFM"] = 0;
	jsonTrajectory["FLASHER"] = 0;
	jsonTrajectory["FMODE"] = 0;
	jsonTrajectory["KERROR"] = 0;
	jsonTrajectory["MODEL_FLAG"] = 0;
	jsonTrajectory["OS"] = 300;
	jsonTrajectory["PATH_A0"] = -0.3;
	jsonTrajectory["PATH_A1"] = -0.08;
	jsonTrajectory["PATH_A2"] = 0.00;
	jsonTrajectory["PATH_A3"] = -1.02;
	jsonTrajectory["PD_NUM"] = 0;
	jsonTrajectory["PREVIEW_DIST"] = 1000;
	jsonTrajectory["PS_NUM"] = 0;
	jsonTrajectory["ROAD_TYPE"] = strOutTrajectory->iRoadType;
	jsonTrajectory["LIGHT_STATUS"] = strOutTrajectory->iLightStatus;
    jsonTrajectory["SAFE_PATH_A0"] = nullptr;
    jsonTrajectory["SAFE_PATH_A1"] = nullptr;
    jsonTrajectory["SAFE_PATH_A2"] = nullptr;
    jsonTrajectory["SAFE_PATH_A3"] = nullptr;
	jsonTrajectory["SDOOR"] = 4000;
	jsonTrajectory["SERRORA"] = 200;
	jsonTrajectory["SERRORHARD"] = 2000;
	jsonTrajectory["SERRORSOFT"] = 800;
	jsonTrajectory["SOT"] = 100;
	jsonTrajectory["KP"] = 800;
	jsonTrajectory["TD_NUM"] = 0;
	jsonTrajectory["TRAJ_LEN"] = 0;
	jsonTrajectory["TS"] = 100;
	jsonTrajectory["TS_NUM"] = 0;
	jsonTrajectory["TURN_LIGHT"] = 0;
	jsonTrajectory["THETA_FF"] = 0;
	jsonTrajectory["KD"] = 0;
	jsonTrajectory["CP"] = 10;
	jsonTrajectory["SP"] = 10;
	jsonTrajectory["KW"] = 3000;
	jsonTrajectory["V"] = 0;
	jsonTrajectory["VP"] = 10;
	jsonTrajectory["VTYPE"] = 400;
	jsonTrajectory["XF"] = 20;
	jsonTrajectory["KPURE"] = 0.0;
	jsonTrajectory["BACK_TARGET_POINT_X"] = 0;
	jsonTrajectory["BACK_TARGET_POINT_Y"] = 0;
	jsonTrajectory["SPOUT_WATER"] = strOutTrajectory->iSpoutWater;
	jsonTrajectory["CLEAN_STATUS"] = strOutTrajectory->iCleanStatus;
	jsonTrajectory["CLEAN_ASH"] = strOutTrajectory->iCleanAsh;
	jsonTrajectory["STATUS"] = strOutTrajectory->iStatus;
	return jsonTrajectory;
}

Json::Value getJsonCarStatus(StrLocationFusion *strCarStatus)
{
	Json::Value jsonCarStatus;
	jsonCarStatus["LATITUDE"] = strCarStatus->dLatitude;
	jsonCarStatus["LONGITUDE"] = strCarStatus->dLongitude;
	jsonCarStatus["ALTITUDE"] = strCarStatus->dAltitude;
	jsonCarStatus["YAW"] = strCarStatus->yaw;
	jsonCarStatus["PITCH"] = strCarStatus->pitch;
	jsonCarStatus["ROLL"] = strCarStatus->roll;
	jsonCarStatus["VEL_EAST"] = strCarStatus->velEast;
	jsonCarStatus["VEL_NORTH"] = strCarStatus->velNorth;
	jsonCarStatus["VEL_UP"] = strCarStatus->velUp;
	jsonCarStatus["GYRO_X"] = strCarStatus->gyroX;
	jsonCarStatus["GYRO_Y"] = strCarStatus->gyroY;
	jsonCarStatus["GYRO_Z"] = strCarStatus->gyroZ;
	return jsonCarStatus;
}

Json::Value getJsonControlConfig(StrControllingConfig *strControlConfig)
{
	Json::Value jsonControlConfig;
	jsonControlConfig["THROTTLE_PUSH_VALUE_HIGH"] = strControlConfig->ThrottlePushValue_high;
	jsonControlConfig["THROTTLE_PUSH_VALUE_LOW"] = strControlConfig->ThrottlePushValue_low;
	jsonControlConfig["SWITH_ACC"] = strControlConfig->Swith_acc;
	jsonControlConfig["KP"] = strControlConfig->Kp;
	jsonControlConfig["KI"] = strControlConfig->Ki;
	jsonControlConfig["KD"] = strControlConfig->Kd;
	jsonControlConfig["BRAKE_SPEED"] = strControlConfig->Brake_speed;
	jsonControlConfig["BRAKE_EMERGENCY"] = strControlConfig->Brake_emergency;
	jsonControlConfig["DT"] = strControlConfig->dt;
	jsonControlConfig["ENABLE_SATURATION_INTEGRATOR"] = strControlConfig->Enable_Saturationintegrator;
	jsonControlConfig["SATURATION_INTEGRATOR_HIGH"] = strControlConfig->f_saturationintegrator_high;
	jsonControlConfig["SATURATION_INTEGRATOR_LOW"] = strControlConfig->f_saturationintegrator_low;
	jsonControlConfig["MAX_HINGE_ANGLE"] = strControlConfig->fmaxhingeangle;
	jsonControlConfig["TOLERANCE"] = strControlConfig->ftolerance;
	jsonControlConfig["MAX_NUM_ITERATION"] = strControlConfig->max_num_iteration;
	jsonControlConfig["X_POLEARM"] = strControlConfig->x_polearm;
	jsonControlConfig["Y_POLEARM"] = strControlConfig->y_polearm;
	jsonControlConfig["Z_POLEARM"] = strControlConfig->z_polearm;
	jsonControlConfig["LAT_ERR_UPPER"] = strControlConfig->lat_err_upper;
	jsonControlConfig["LAT_ERR_LOWER"] = strControlConfig->lat_err_lower;
	jsonControlConfig["LAT_SPEED_ERR_UPPER"] = strControlConfig->lat_speed_err_upper;
	jsonControlConfig["LAT_SPEED_ERR_LOWER"] = strControlConfig->lat_speed_err_lower;
	jsonControlConfig["BRAKE_PUSH_VALUE"] = strControlConfig->BrakePushValue;
	return jsonControlConfig;
}

Json::Value getJsonReferenceLine(Global *strReferenceLine)
{
	Json::Value jsonGlobal;
	jsonGlobal["NUM_POINT"] = strReferenceLine->NUM_POINT;
	Json::Value points;
	for (int i = 0; i < strReferenceLine->NUM_POINT; i++)
	{
		Json::Value point;
		point["ID"] = strReferenceLine->POINT[i].ID;
		point["X"] = strReferenceLine->POINT[i].X;
		point["Y"] = strReferenceLine->POINT[i].Y;
		points.append(point);
	}
	jsonGlobal["REFERENCE_POINTS"] = points;
	jsonGlobal["L0"] = strReferenceLine->L0;
	jsonGlobal["B0"] = strReferenceLine->B0;
	jsonGlobal["V"] = strReferenceLine->V;
	jsonGlobal["TYPE_ROAD"] = strReferenceLine->TYPE_ROAD;
	return jsonGlobal;
}

Json::Value getJsonCarStatus4Debug(StrLocationFusion *strCarStatus, SLPoint *VehicleSLPoint, StrPlanningConfig *strPlanningConfig)
{
	Json::Value jsonCarStatus;
	jsonCarStatus["LATITUDE"] = strCarStatus->dLatitude;
	jsonCarStatus["LONGITUDE"] = strCarStatus->dLongitude;
	jsonCarStatus["ALTITUDE"] = strCarStatus->dAltitude;
	jsonCarStatus["ENU_X"] = strCarStatus->fX;
	jsonCarStatus["ENU_Y"] = strCarStatus->fY;
	jsonCarStatus["ENU_Z"] = strCarStatus->fZ;
	jsonCarStatus["S"] = VehicleSLPoint->fS;
	jsonCarStatus["L"] = VehicleSLPoint->fL;
	jsonCarStatus["VEHICLE_WIDTH"] = strPlanningConfig->fVehicleWidth;
	jsonCarStatus["VEHICLE_LENGTH"] = strPlanningConfig->fVehicleLength;
	return jsonCarStatus;
}

Json::Value getJsonReferenceLine4Debug(Global *strReferenceLine)
{
	Json::Value jsonReferenceLine;
	jsonReferenceLine["NUM_REFERENCE_POINTS"] = strReferenceLine->NUM_POINT;
	Json::Value referencePoints;
	for (int i = 0; i < strReferenceLine->NUM_POINT; i++)
	{
		Json::Value referencePoint;
		referencePoint["ID"] = strReferenceLine->POINT[i].ID;
		referencePoint["X"] = strReferenceLine->POINT[i].X;
		referencePoint["Y"] = strReferenceLine->POINT[i].Y;
		referencePoints.append(referencePoint);
	}
	jsonReferenceLine["REFERENCE_POINTS"] = referencePoints;
	return jsonReferenceLine;
}

Json::Value getJsonTargetPoints4Debug(StrSLPoints *TargetSLPoints)
{
	Json::Value jsonTargetPoints;
	jsonTargetPoints["LEFT_TARGET_NUM"] = LeftTargetNum;
	if (LeftTargetNum > 0)
	{
		Json::Value leftTargetPoint;
		leftTargetPoint["S"] = TargetSLPoints->pSLPoint[0].fS;
		leftTargetPoint["L"] = TargetSLPoints->pSLPoint[0].fL;
		leftTargetPoint["X"] = TargetSLPoints->pSLPoint[0].fX;
		leftTargetPoint["Y"] = TargetSLPoints->pSLPoint[0].fY;
		leftTargetPoint["WIDTH"] = TargetSLPoints->pSLPoint[0].fWidth;
		leftTargetPoint["LENGTH"] = TargetSLPoints->pSLPoint[0].fLength;
		jsonTargetPoints["LEFT_TARGET_POINT"] = leftTargetPoint;
	}
	jsonTargetPoints["RIGHT_TARGET_NUM"] = RightTargetNum;
	if (RightTargetNum > 0)
	{
		Json::Value rightTargetPoint;
		rightTargetPoint["S"] = TargetSLPoints->pSLPoint[1].fS;
		rightTargetPoint["L"] = TargetSLPoints->pSLPoint[1].fL;
		rightTargetPoint["X"] = TargetSLPoints->pSLPoint[1].fX;
		rightTargetPoint["Y"] = TargetSLPoints->pSLPoint[1].fY;
		rightTargetPoint["WIDTH"] = TargetSLPoints->pSLPoint[1].fWidth;
		rightTargetPoint["LENGTH"] = TargetSLPoints->pSLPoint[1].fLength;
		jsonTargetPoints["RIGHT_TARGET_POINT"] = rightTargetPoint;
	}
	return jsonTargetPoints;
}

Json::Value getJsonDecisionPoint4Debug(SLPoint *DecisionPoint)
{
	Json::Value jsonDecisionPoint;
	jsonDecisionPoint["X"] = DecisionPoint->fX;
	jsonDecisionPoint["Y"] = DecisionPoint->fY;
	jsonDecisionPoint["S"] = DecisionPoint->fS;
	jsonDecisionPoint["L"] = DecisionPoint->fL;
	return jsonDecisionPoint;
}

Json::Value getJsonDecisionState4Debug(EmDecision *Decision, EmDecision *UltrasonicDecision, EmDecision *RadarDecision)
{
	Json::Value jsonDecisionState;
	jsonDecisionState["DECISION"] = Decision;
	jsonDecisionState["ULTRASONIC_DECISION"] = UltrasonicDecision;
	jsonDecisionState["RADAR_DECISION"] = RadarDecision;
	return jsonDecisionState;
}

Json::Value getJsonTrajectory4Debug(StrSLPoints *strOutTrajectory)
{
	Json::Value jsonTrajectory;
	jsonTrajectory["A0"] = strOutTrajectory->pfC[0];
	jsonTrajectory["A1"] = strOutTrajectory->pfC[1];
	jsonTrajectory["A2"] = strOutTrajectory->pfC[2];
	jsonTrajectory["A3"] = strOutTrajectory->pfC[3];
	jsonTrajectory["SX"] = PlanOutStartPoint.X;
	jsonTrajectory["SY"] = PlanOutStartPoint.Y;
	jsonTrajectory["EX"] = PlanOutEndPoint.X;
	jsonTrajectory["EY"] = PlanOutEndPoint.Y;
	return jsonTrajectory;
}

Json::Value getJsonCleaningOperation4Debug(StrSLPoints *strOutTrajectory)
{
	Json::Value jsonCleaningOperation;
	jsonCleaningOperation["LIGHT_STATUS"] = strOutTrajectory->iLightStatus;
	jsonCleaningOperation["SPOUT_WATER"] = strOutTrajectory->iSpoutWater;
	jsonCleaningOperation["CLEAN_STATUS"] = strOutTrajectory->iCleanStatus;
	jsonCleaningOperation["CLEAN_ASH"] = strOutTrajectory->iCleanAsh;
	return jsonCleaningOperation;
}

#if 0
Json::Value getJsonPlanningConfig(StrPlanningConfig *strPlanningConfig)
{
	Json::Value jsonPlanningConfig;
	jsonPlanningConfig["SAFE_DISTANCE_1"] = strPlanningConfig->fSafeDis1;
	jsonPlanningConfig["SAFE_DISTANCE_2"] = strPlanningConfig->fSafeDis2;
	jsonPlanningConfig["MAX_PLANNING_DISTANCE"] = strPlanningConfig->fMaxPlanningDistance;
	jsonPlanningConfig["MAX_ACC"] = strPlanningConfig->fMaxAcc;
	jsonPlanningConfig["MAX_V"] = strPlanningConfig->fMaxV;
	jsonPlanningConfig["FOLLOW_DISTANCE"] = strPlanningConfig->follow_dis;
	jsonPlanningConfig["ULTRASONIC_SPEED"] = strPlanningConfig->fUltrasonicSpeed;
	jsonPlanningConfig["ULTRASONIC_DISTANCE"] = strPlanningConfig->fUltrasonicDis;
	jsonPlanningConfig["VEHICLE_WIDTH"] = strPlanningConfig->fVehicleWidth;
	jsonPlanningConfig["VEHICLE_LENGTH"] = strPlanningConfig->fVehicleLength;
	jsonPlanningConfig["SAMPLE_BUFFER"] = strPlanningConfig->fSampleBuffer;
	jsonPlanningConfig["SAMPLE_ACCURACY"] = strPlanningConfig->fSampleAccuracy;
	jsonPlanningConfig["SAMPLE_TIME"] = strPlanningConfig->fSampleTime;
	jsonPlanningConfig["UNIT_S"] = strPlanningConfig->fUnitS;
	jsonPlanningConfig["QUASI_SOFTMAX_L0"] = strPlanningConfig->fl0;
	jsonPlanningConfig["QUASI_SOFTMAX_B"] = strPlanningConfig->fb;
	jsonPlanningConfig["QUASI_SOFTMAX_K"] = strPlanningConfig->fk;
	jsonPlanningConfig["PATH_COST"] = strPlanningConfig->fPathLCost;
	jsonPlanningConfig["PATH_DL_COST"] = strPlanningConfig->fPathDLCost;
	jsonPlanningConfig["PATH_DDL_COST"] = strPlanningConfig->fPathDDLCost;
	jsonPlanningConfig["END_COST"] = strPlanningConfig->fEndCost;
	jsonPlanningConfig["NUDGE_BUFFER"] = strPlanningConfig->fNudgeBuffer;
	jsonPlanningConfig["IGNORE_L"] = strPlanningConfig->fIgnoreL;
	jsonPlanningConfig["OBSTACLE_COLLISION_COST"] = strPlanningConfig->fObstacleCollisionCost;
	jsonPlanningConfig["OBSTACLE_COLLOSION_DISTANCE"] = strPlanningConfig->fObstacleCollisionDis;
	jsonPlanningConfig["OBSTACLE_BUFFER"] = strPlanningConfig->fObstacleBuffer;
	jsonPlanningConfig["LEFT_HALF_ROAD_WIDTH"] = strPlanningConfig->fLeftHalfRoadWidth;
	jsonPlanningConfig["RIGHT_HALF_ROAD_WIDTH"] = strPlanningConfig->fRightHalfRoadWidth;
	jsonPlanningConfig["ORIGIN_LATITUDE"] = strPlanningConfig->dOriginLat;
	jsonPlanningConfig["ORIGIN_LONGITUDE"] = strPlanningConfig->dOriginLon;
	jsonPlanningConfig["ORIGIN_ALTITUDE"] = strPlanningConfig->dOriginAlt;
	jsonPlanningConfig["RECORD_FILE"] = strPlanningConfig->iRecordFile;
	jsonPlanningConfig["READ_PLANNING_CONFIG_SUCCESSFUL"] = strPlanningConfig->ReadPlanningConfig;
	return jsonPlanningConfig;
}
#endif

// X*A=Y, X:[5,4],A:[4,1],Y:[5,1]
// 采用最小均方差的方式求解A: A=(X^T * X)^(-1) * X^(T) * Y
void MatrixSlove(int n, double XI[], double YI[], double *A)
{
	int i = 0;
	int j = 0;

	Eigen::MatrixXd X(n, 4);
	Eigen::MatrixXd Y(n, 1);

	Eigen::MatrixXd R0(4, 4);
	Eigen::MatrixXd R(4, 1);

	// 矩阵初始化
	for (i = 0; i < n; i++)
	{
		X(i, 0) = 1.0;
		X(i, 1) = XI[i];
		X(i, 2) = XI[i] * XI[i];
		X(i, 3) = XI[i] * XI[i] * XI[i];
		Y(i, 0) = YI[i];
	}

	Eigen::MatrixXd X1(4, n);
	X1 = X.transpose();
	R0 = X.transpose() * X;
	Eigen::MatrixXd R2(4, 4);
	R2 = R0.inverse();
	R = R0.inverse() * X.transpose() * Y;
	for (i = 0; i < 4; i++)
		A[i] = R(i, 0);

	return;
}

// 采用最小均方差的方式，使用5个方程求解三阶曲线的四个参数
void polyfit2(int Num, double X[], double Y[], double *pA)
{
	int i = 0;
	double A[4] = { 0.0 };
	MatrixSlove(Num, X, Y, A);
	for (i = 0; i < 4; i++)
		pA[i] = A[i];
	return;
}
int iPlanning(StrLocationFusion *pStrCarStatus, StrTargetFusion *pTargetsFusion, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pStrOutTrajectory)
{
	int iRet = 0;
	int i = 0;
	int j = 0;

	if (NULL == pStrCarStatus || NULL == pTargetsFusion || NULL == pStrPlanningConfig)
	{
		if (NULL == pStrCarStatus)
			printf("本车定位数据为空 \n");
		else if (NULL == pTargetsFusion)
			printf("障碍物信息为空 \n");
		else if (NULL == pStrPlanningConfig)
			printf("规划模块的配置参数为空 \n");
		return -1;
	}

	if (NULL == pStrOutTrajectory)
	{
		printf("规划输出的初始化失败 \n");
		return -1;
	}

	iRet = iReferenceProvider(&HdMap, pStrCarStatus, pStrPlanningConfig, &StrReferenceLine);
	if (-1 == iRet)
	{
		pStrOutTrajectory->iStatus = PLAN_FAILED;
		printf("获取车附近的参考线失败 \n");
		return -1;
	}

	// 判断是否有超声波雷达数据,定位系统是否可靠以及决策输出退出安全系统以及安全停车指令
	if (pStrCarStatus->cSystemStatus != 1 || 1 == StrReferenceLine.PARK)
	{
		if (pStrCarStatus->cSystemStatus != 1)
			pStrOutTrajectory->iStatus = LOACTION_INVALID;		// 定位不准确
		else
			pStrOutTrajectory->iStatus = QUIT_SYSTEM;			// 退出系统

		pStrOutTrajectory->iNum = StrReferenceLine.NUM_POINT;
		// 刹车，使用车辆当前位置作为所有规划输出点的结果
		for (i = 0; i < pStrOutTrajectory->iNum; i++)
		{
			pStrOutTrajectory->pSLPoint[i].iId = i;
			pStrOutTrajectory->pSLPoint[i].fX = pStrCarStatus->fX;
			pStrOutTrajectory->pSLPoint[i].fY = pStrCarStatus->fY;
			pStrOutTrajectory->pSLPoint[i].fZ = pStrCarStatus->fZ;
			pStrOutTrajectory->pSLPoint[i].fTheta = pStrCarStatus->fAngle;		// 这个角度是否正确?如果是航向角的话，这样赋值OK，即保持当前的朝向；如果是偏转角的话，这样赋值不行
			pStrOutTrajectory->pSLPoint[i].fKappa = pStrCarStatus->fKappa;
			pStrOutTrajectory->pSLPoint[i].fV = 0.0;
			pStrOutTrajectory->pSLPoint[i].fA = -pStrPlanningConfig->fMaxAcc;	// 实际上，只要前面的几个加速度会起作用，就可以把车刹住；如果一直减速会不会倒退，要看一下控制模块
		}

		return 0;
	}

	// 把地图上参考点的坐标从车体坐标系转换到SL坐标系，同时，继承了车体坐标系下的XYZ坐标
	iRet = iReferLineXYToSL(&StrReferenceLine, pStrCarStatus, &StrSLReferLine);
	if (-1 == iRet)
	{
		printf("ReferLine XY To SL Failed\n");
		return -1;
	}

	StrRoutePaths StrRoutesPaths;
	iRet = iRoutePlanner(&StrSLReferLine, pStrCarStatus, pTargetsFusion, pStrPlanningConfig, &StrRoutesPaths);
	if (-1 == iRet)			// 规划失败
	{
		printf("路径规划失败 \n");
		pStrOutTrajectory->iStatus = PLAN_FAILED;
		pStrOutTrajectory->iNum = StrReferenceLine.NUM_POINT;
		for (i = 0; i < pStrOutTrajectory->iNum; i++)
		{
			pStrOutTrajectory->pSLPoint[i].iId = i;
			pStrOutTrajectory->pSLPoint[i].fX = 0.0;
			pStrOutTrajectory->pSLPoint[i].fY = 0.0;
			pStrOutTrajectory->pSLPoint[i].fZ = 0.0;
			pStrOutTrajectory->pSLPoint[i].fTheta = pStrCarStatus->fAngle;		// 偏转角，是不是应该赋值为0
			pStrOutTrajectory->pSLPoint[i].fKappa = pStrCarStatus->fKappa;
			pStrOutTrajectory->pSLPoint[i].fV = 0.0;
			pStrOutTrajectory->pSLPoint[i].fA = pStrCarStatus->fAcceleration;	// 加速度，是不是应该赋值为0
		}
		return -1;
	}
	else if (1 == iRet)		// 安全停车
	{
		pStrOutTrajectory->iStatus = Safestop;
		pStrOutTrajectory->iNum = StrReferenceLine.NUM_POINT;
		for (i = 0; i < pStrOutTrajectory->iNum; i++)
		{
			pStrOutTrajectory->pSLPoint[i].iId = i;
			pStrOutTrajectory->pSLPoint[i].fX = 0.0;
			pStrOutTrajectory->pSLPoint[i].fY = 0.0;
			pStrOutTrajectory->pSLPoint[i].fZ = 0.0;
			pStrOutTrajectory->pSLPoint[i].fTheta = 0.0;
			pStrOutTrajectory->pSLPoint[i].fKappa = 0.0;
			pStrOutTrajectory->pSLPoint[i].fV = 0.0;
			pStrOutTrajectory->pSLPoint[i].fA = 0.0;
		}
		return -1;
	}

	// 三次曲线拟合
    Point TmpPoint = { 0, 0.0, 0.0, 0.0 };
	MapPoint TMapPoint;
	double dP[4] = { 0.0 };
	double dX[MAX_SLPOINT_NUM] = { 0.0 };
	double dY[MAX_SLPOINT_NUM] = { 0.0 };
	int iNum = 5;	// 拟合点数

	// 添加上一帧规划点序列中的前三个点数据
	// 转换到当前车体坐标系，然后再进行拟合
	if (1 == PlanStatus)
	{
		for (i = 0; i < 3; i++)
		{
			TMapPoint.dX = HisPlanPoints.pSLPoint[i + HisId].fX;
			TMapPoint.dY = HisPlanPoints.pSLPoint[i + HisId].fY;
			TMapPoint.dZ = HisPlanPoints.pSLPoint[i + HisId].fZ;
			TmpPoint = Enu2Rfu(pStrCarStatus, &TMapPoint);
			dX[i] = TmpPoint.X;
			dY[i] = TmpPoint.Y;
		}

		for (j = 0; j < iNum - 3; j++)
		{
			dX[j + 3] = StrRoutesPaths.strRoutePath[0].pSLPoint[j].fX;
			dY[j + 3] = StrRoutesPaths.strRoutePath[0].pSLPoint[j].fY;
		}
	}
	else
	{
		dX[0] = 0.0;
		dY[0] = 0.0;
		for (j = 1; j < iNum; j++)
		{
			dX[j] = StrRoutesPaths.strRoutePath[0].pSLPoint[j - 1].fX;
			dY[j] = StrRoutesPaths.strRoutePath[0].pSLPoint[j - 1].fY;
		}
	}

	PlanOutStartPoint.X = dX[0];
	PlanOutStartPoint.Y = dY[0];
	PlanOutEndPoint.X = dX[iNum - 1];
	PlanOutEndPoint.Y = dY[iNum - 1];

	// 采用最小均方差的方式，使用5个方程(X * A = Y)求解三阶曲线的四个参数(a0, a1, a2, a3)
	// y = a0 + a1 * x + a2 * x^2 + a3 * x^3
	polyfit2(iNum, dX, dY, dP);
	if (fabs(dP[0]) >= 100.000 || fabs(dP[1]) >= 100.000 || fabs(dP[2]) >= 100.000 || fabs(dP[3]) >= 100.000)
	{
		dP[0] = 0.0;
		dP[1] = 0.0;
		dP[2] = 0.0;
		dP[3] = 0.0;
		printf("三次多项式的系数拟合错误 \n");
		return -1;
	}

	int k = 0;
	for (k = 0; k < 4; k++)
		StrRoutesPaths.strRoutePath[0].pfC[k] = dP[k];

	// 在每一帧规划结束后，把本帧规划的轨迹点保存起来放在HisPlanPoints中，需要把HisPlanPoints中规划点的坐标保持在ENU坐标系下
    Point TPoint = { 0, 0.0, 0.0, 0.0 };
	MapPoint PlanPoints;
	if (1 == PlanStatus)
	{
		HisPlanPoints.iNum = StrRoutesPaths.strRoutePath[0].iNum + 3;
		// 取上一帧规划的三个轨迹点作为当前帧规划的前三个点
		for (i = 0; i < 3; i++)
		{
			HisPlanPoints.pSLPoint[i].iId = i;
			HisPlanPoints.pSLPoint[i].fX = HisPlanPoints.pSLPoint[i + HisId].fX;
			HisPlanPoints.pSLPoint[i].fY = HisPlanPoints.pSLPoint[i + HisId].fY;
			HisPlanPoints.pSLPoint[i].fZ = HisPlanPoints.pSLPoint[i + HisId].fZ;
			// 下面这句话等于什么都没做，自己赋值给自己。即使如此，我们是否需要更新三个轨迹点的S坐标?
			// 如果不更新的话，当参考点之间的S坐标差值是恒定时，倒也关系不大；如果要求精确的话，就需要更新了
			HisPlanPoints.pSLPoint[i].fS = HisPlanPoints.pSLPoint[i].fS;
			HisPlanPoints.pSLPoint[i].fL = HisPlanPoints.pSLPoint[i + HisId].fL;
			HisPlanPoints.pSLPoint[i].fTheta = HisPlanPoints.pSLPoint[i + HisId].fTheta;
		}

		for (i = 3; i < StrRoutesPaths.strRoutePath[0].iNum + 3; i++)
		{
			HisPlanPoints.pSLPoint[i].iId = i;
			HisPlanPoints.pSLPoint[i].fX = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fX;
			HisPlanPoints.pSLPoint[i].fY = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fY;
			HisPlanPoints.pSLPoint[i].fZ = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fZ;
			HisPlanPoints.pSLPoint[i].fS = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fS;	// 这里的S坐标是否可以和前面3个轨迹点的S坐标衔接上，好像可以
			HisPlanPoints.pSLPoint[i].fL = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fL;
			HisPlanPoints.pSLPoint[i].fTheta = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 3].fTheta;

			// RFU转ENU，在每帧规划结束后，HisPlanPoints中点的XYZ坐标都是在ENU坐标系下的
			PlanPoints.dX = HisPlanPoints.pSLPoint[i].fX;
			PlanPoints.dY = HisPlanPoints.pSLPoint[i].fY;
			PlanPoints.dZ = HisPlanPoints.pSLPoint[i].fZ;
			TPoint = Rfu2Enu(pStrCarStatus, &PlanPoints);
			HisPlanPoints.pSLPoint[i].iId = i;
			HisPlanPoints.pSLPoint[i].fX = TPoint.X;
			HisPlanPoints.pSLPoint[i].fY = TPoint.Y;
			HisPlanPoints.pSLPoint[i].fZ = TPoint.Z;
		}
	}
	else
	{
		HisPlanPoints.iNum = StrRoutesPaths.strRoutePath[0].iNum + 1;
		HisPlanPoints.pSLPoint[0].iId = 0;
		HisPlanPoints.pSLPoint[0].fX = pStrCarStatus->fX;										// 这是ENU坐标系下的XYZ坐标
		HisPlanPoints.pSLPoint[0].fY = pStrCarStatus->fY;
		HisPlanPoints.pSLPoint[0].fZ = pStrCarStatus->fZ;
		HisPlanPoints.pSLPoint[0].fS = VehicleSLPoint.fS;
		HisPlanPoints.pSLPoint[0].fL = VehicleSLPoint.fL;
		HisPlanPoints.pSLPoint[0].fTheta = 0.0;
		for (i = 1; i < StrRoutesPaths.strRoutePath[0].iNum + 1; i++)
		{
			HisPlanPoints.pSLPoint[i].iId = i;
			HisPlanPoints.pSLPoint[i].fX = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fX;
			HisPlanPoints.pSLPoint[i].fY = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fY;
			HisPlanPoints.pSLPoint[i].fZ = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fZ;
			HisPlanPoints.pSLPoint[i].fS = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fS;	// 这里的S坐标是否可以和前面1个轨迹点的S坐标衔接上，好像可以
			HisPlanPoints.pSLPoint[i].fL = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fL;
			HisPlanPoints.pSLPoint[i].fTheta = StrRoutesPaths.strRoutePath[0].pSLPoint[i - 1].fTheta;

			// RFU坐标系转成ENU坐标系，在每帧规划结束后，HisPlanPoints中点的XYZ坐标是在ENU坐标系下的
			PlanPoints.dX = HisPlanPoints.pSLPoint[i].fX;
			PlanPoints.dY = HisPlanPoints.pSLPoint[i].fY;
			PlanPoints.dZ = HisPlanPoints.pSLPoint[i].fZ;
			TPoint = Rfu2Enu(pStrCarStatus, &PlanPoints);
			HisPlanPoints.pSLPoint[i].iId = i;
			HisPlanPoints.pSLPoint[i].fX = TPoint.X;
			HisPlanPoints.pSLPoint[i].fY = TPoint.Y;
			HisPlanPoints.pSLPoint[i].fZ = TPoint.Z;
		}
	}

	// 当规划正常时，把PlanStatus标志位置为1
	PlanStatus = 1;

	// 保存当前帧的50个参考点数据ReferencePoint到HisReferencePoint中
	// 保存当前帧的50个参考点数据StrSLReferLine到HisStrSLReferLine中
	for (i = 0; i < 50; i++)
	{
		HisReferencePoint[i] = ReferencePoint[i];
		HisStrSLReferLine.iNum = 50;
		HisStrSLReferLine.pSLPoint[i] = StrSLReferLine.pSLPoint[i];
	}

	if (Decision == Forward && (HisDecision == LeftPass || HisDecision == RightPass))		// 上一帧的决策是绕行，而这一帧的决策是直行
	{
		if (debug)
			printf("上一帧的决策是绕行，而当前帧的决策是直行 \n");
	}

	// 速度规划
	iRet = iSpeedPlanner(&StrRoutesPaths, pStrPlanningConfig);
	if (-1 == iRet)
	{
		printf("速度规划失败 \n");
		return -1;
	}

	// 代价选择
	iRet = iCostSelect(&StrRoutesPaths, pTargetsFusion, &StrSLReferLine, pStrPlanningConfig, pStrOutTrajectory);
	if (-1 == iRet)
	{
		printf("代价选择失败 \n");
		return -1;
	}

	return 0;
}

int iJsonToRerLine(Json::Value RecvGlobal, Global *pStrReferenceLine)
{
	int i = 0;

	if (NULL == pStrReferenceLine)
	{
		printf("Input param Error\n");
		return -1;
	}

	Json::Value Points;
	pStrReferenceLine->NUM_POINT = RecvGlobal["NUM_POINT"].asInt();
	Points = RecvGlobal["POINT"];
	for (i = 0; i < Points.size(); i++)
	{
		Json::Value Point = Points[i];
		pStrReferenceLine->POINT[i].ID = Point["ID"].asDouble();
		pStrReferenceLine->POINT[i].X = Point["X"].asDouble();
		pStrReferenceLine->POINT[i].Y = Point["Y"].asDouble();
	}

	pStrReferenceLine->L0 = RecvGlobal["L0"].asDouble();
	pStrReferenceLine->B0 = RecvGlobal["B0"].asDouble();
	pStrReferenceLine->V = (float)RecvGlobal["V"].asDouble();
	pStrReferenceLine->TYPE_ROAD = RecvGlobal["TYPE_ROAD"].asInt();

	return 0;
}

int iJsonToCarStatus(Json::Value RecvGPS, StrLocationFusion *pStrCarStatus)
{
	if (NULL == pStrCarStatus)
	{
		printf("Output param Error\n");
		return -1;
	}

	pStrCarStatus->fX = 0.0;
	pStrCarStatus->fY = 0.0;
	pStrCarStatus->fZ = 0.0;
	pStrCarStatus->dAltitude = RecvGPS["ALTITUDE"].asDouble();
	pStrCarStatus->dLatitude = RecvGPS["LATITUDE"].asDouble();;
	pStrCarStatus->dLongitude = RecvGPS["LONGTITUDE"].asDouble();
	pStrCarStatus->fSpeed = RecvGPS["SPEED"].asFloat();
	pStrCarStatus->fAcceleration = RecvGPS["ACCELERATION"].asFloat();
	pStrCarStatus->fAngle = RecvGPS["YAW"].asFloat();
	pStrCarStatus->yaw = pStrCarStatus->fAngle;
	pStrCarStatus->pitch = RecvGPS["PITCH"].asFloat();
	pStrCarStatus->roll = RecvGPS["ROLL"].asFloat();
	pStrCarStatus->fAngleRate = RecvGPS["ANGLE_RATE"].asFloat();
	pStrCarStatus->velEast = RecvGPS["VEL_EAST"].asFloat();
	pStrCarStatus->velNorth = RecvGPS["VEL_NORTH"].asFloat();
	pStrCarStatus->velUp = RecvGPS["VEL_UP"].asFloat();
	pStrCarStatus->gyroX = RecvGPS["GYRO_X"].asFloat();
	pStrCarStatus->gyroY = RecvGPS["GYRO_Y"].asFloat();
	pStrCarStatus->gyroZ = RecvGPS["GYRO_Z"].asFloat();
	string NAV_Status = RecvGPS["NAV_STATE"].asString();
	string GPS_Status = RecvGPS["GPS_STATE"].asString();

    // port to linux, and changed
    //int iNavStatus = (int)NAV_Status.c_str();
    //int iGpsStatus = (int)GPS_Status.c_str();
    int iNavStatus = std::stoi(NAV_Status);
    int iGpsStatus = std::stoi(GPS_Status.c_str());

	if ((iNavStatus >= 1) && (iGpsStatus >= 1))
		pStrCarStatus->cSystemStatus = 1;
	else
		pStrCarStatus->cSystemStatus = 0;

	return 0;
}

int iJsonToTargetsStatus(Json::Value RecvSense, StrTargetFusion *pTargetsFusion)
{
	int i = 0;
	if (NULL == pTargetsFusion)
	{
		printf("Output param Error\n");
		return -1;
	}
	pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fX = RecvSense["CURB"]["LEFT"][0]["X"].asDouble();
	pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fY = RecvSense["CURB"]["LEFT"][0]["Y"].asDouble();
	pTargetsFusion->strRoadBoard.strLeftRoadBoard[1].fX = RecvSense["CURB"]["LEFT"][1]["X"].asDouble();
	pTargetsFusion->strRoadBoard.strLeftRoadBoard[1].fY = RecvSense["CURB"]["LEFT"][1]["Y"].asDouble();
	pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fX = RecvSense["CURB"]["RIGHT"][0]["X"].asDouble();
	pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fY = RecvSense["CURB"]["RIGHT"][0]["Y"].asDouble();
	pTargetsFusion->strRoadBoard.strRightRoadBoard[1].fX = RecvSense["CURB"]["RIGHT"][1]["X"].asDouble();
	pTargetsFusion->strRoadBoard.strRightRoadBoard[1].fY = RecvSense["CURB"]["RIGHT"][1]["Y"].asDouble();
	pTargetsFusion->strRoadBoard.iRoadPointNum = 2;

	Json::Value recv_ultrasound = RecvSense["ULTRASOUND_OBJECTS"];

	int iRadarNum = RecvSense["RADAR_OBJECTS"].size();
	int iLidarNum = RecvSense["LIDAR_OBJECTS"].size();

	if (iRadarNum + iLidarNum == 0)
		pTargetsFusion->iTargetNum = 0;

	// 障碍物数量超出MAX_TARGET_NUM
	if (iRadarNum + iLidarNum >= MAX_TARGET_NUM)
		iLidarNum = MAX_TARGET_NUM - iRadarNum;
	pTargetsFusion->iTargetNum = iRadarNum + iLidarNum;

	Json::Value RadarObjects = RecvSense["RADAR_OBJECTS"];
	for (i = 0; i < iRadarNum; i++)
	{
		Json::Value Object = RadarObjects[i];
		pTargetsFusion->strTargetStatus[i].emSensorType = SENSOR_RADAR;
		pTargetsFusion->strTargetStatus[i].fX = Object["X"].asFloat();
		pTargetsFusion->strTargetStatus[i].fY = Object["Y"].asFloat();
		pTargetsFusion->strTargetStatus[i].iTargetId = iRadarNum + i;
		pTargetsFusion->strTargetStatus[i].fRate = Object["RR"].asFloat();
		pTargetsFusion->strTargetStatus[i].fAngle = Object["A"].asFloat();
		pTargetsFusion->strTargetStatus[i].fAngleRate = Object["LR"].asFloat();
		pTargetsFusion->strTargetStatus[i].fWidth = Object["W"].asFloat();
		pTargetsFusion->strTargetStatus[i].fLength = Object["L"].asFloat();
	}

	Json::Value LidarObjects = RecvSense["LIDAR_OBJECTS"];
	for (i = 0; i < iLidarNum; i++)
	{
		Json::Value Object = LidarObjects[i];
		pTargetsFusion->strTargetStatus[i + iRadarNum].emSensorType = SENSOR_LIDAR;
		pTargetsFusion->strTargetStatus[i + iRadarNum].fX = Object["X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].fY = Object["Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].iTargetId = iRadarNum + i;
		pTargetsFusion->strTargetStatus[i + iRadarNum].fRate = Object["SX"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].fAngle = Object["A"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].fWidth = Object["W"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].fLength = Object["L"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[0].fX = Object["P1_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[0].fY = Object["P1_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[1].fX = Object["P2_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[1].fY = Object["P2_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[2].fX = Object["P3_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[2].fY = Object["P3_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[3].fX = Object["P4_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iRadarNum].strTargetBoard.strTargetPoint[3].fY = Object["P4_Y"].asFloat();
	}

	pTargetsFusion->strUltrasonicDatas.byHaveUltrasonic = false;
	for (i = 0; i < ULTRASOUND_NUM; i++)
	{
		Json::Value UltrasoudObjects = recv_ultrasound[i];
		pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].iId = UltrasoudObjects["POSITION"].asInt();
		pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].fDis = UltrasoudObjects["DISTANCE"].asFloat();
		pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].UltrasonicAvailable = false;

		if (true == UltrasoudObjects["AVAILABLE"].asBool() && (pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].fDis > 0.0 && pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].fDis <= 5.5) && (UltrasoudObjects["POSITION"].asInt() < 4))
			pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].UltrasonicAvailable = true;

		if (pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].UltrasonicAvailable == true)
			pTargetsFusion->strUltrasonicDatas.byHaveUltrasonic = true;

		if (pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].UltrasonicAvailable == true)
		{
			if (debug)
				printf("第%d号超声波检测到障碍物，距离为%f\n", pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].iId, pTargetsFusion->strUltrasonicDatas.UltrasonicData[i].fDis);
		}
	}

	return 0;
}

int iPlanningProcessInit(void)
{
	int i = 0;
	int j = 0;
	int iRet = 0;
	char pchDir1[1024] = "./config/planning.cfg";
	char pchDir2[1024] = "./config/controlling.cfg";
	char pchDir3[1024] = "./config/hdmap.bin";

	// 初始化规划接口，获取配置参数
	iRet = iPlanningInit(pchDir1, &strPlanningConfig);
	if (-1 == iRet)
	{
		printf("Planning Init Error\n");
		strPlanningConfig.ReadPlanningConfig = false;
	}
	else
		strPlanningConfig.ReadPlanningConfig = true;

	if (1 == strPlanningConfig.iRecordFile)
	{
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c),
                            "./PlaningOutput/data_input_%Y%m%d%H%M%S.txt");
        std::string strFileName = ss.str();

        pFd0 = fopen(strFileName.c_str(), "wt+");
		if (NULL == pFd0)
		{
			printf("Open Data_Input File Failed\n");
			return -1;
		}

        ss.clear();
        ss << std::put_time(std::localtime(&now_c),
                            "./PlaningOutput/data_output_%Y%m%d%H%M%S.txt");
        strFileName = ss.str();
        pFd1 = fopen(strFileName.c_str(), "wt+");
		if (NULL == pFd1)
		{
			printf("Open Data_OutPut File Failed\n");
			return -1;
		}
	}


	// 读取控制模块配置参数
	iRet = iReadControlConfig(pchDir2, &strControlConfig);
	if (-1 == iRet)
	{
		printf("Read Control Param Error\n");
		strControlConfig.ReadControlConfig = false;
	}
	else
		strControlConfig.ReadControlConfig = true;

	// 读取高精地图文件
	iRet = iReadHdMap(pchDir3, &HdMap);
	if (-1 == iRet)
	{
		printf("Read HdMap Failed\n");
		HdMap.ReadHdMap = false;
		return -1;
	}
	else
		HdMap.ReadHdMap = true;

	return 0;
}

int main()
{
	printf("Enter the Local Planning Module\n");

	LogHelpInit();
	iPlanningProcessInit();
	AlgExcTimer Timer0;
	AlgExcTimer Timer1;
	AlgExcTimer Timer2;

	// 线程1：局部路径规划模块向DataPool请求输入数据
	threads.push_back(thread([&]()
	{
		void* context = zmq_init(1);
		void* z_socket = zmq_socket(context, ZMQ_REQ);
		zmq_connect(z_socket, "tcp://127.0.0.1:5556");
		int send_status = 0;

		while (true)
		{
			// 向DataPool请求输入数据
			if (send_status == 0)
			{
#if 0
				unique_lock <mutex> lock_input_request(g_mutex_input_request);
				while (!g_flag_input_request)
				{
					if (debug)
						printf("等待唤醒请求数据的线程 \n");
					g_cv_input_request.wait(lock_input_request);
				}
				g_flag_input_request = false;
#endif
				Timer1.StartTimer();
				// 将请求打包成Json格式
				Json::Value req;
				Json::StreamWriterBuilder writerBuilder;
				ostringstream os;
				string str_send;
				req["TYPE"] = "REQ_LOCAL";
				unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
				jsonWriter->write(req, &os);
				str_send = os.str();

				// 将Json格式数据发送给DataPool
				int length = (int)str_send.length();
				int iRet = zmq_send(z_socket, str_send.c_str(), length, 0);
				if (iRet != length)
				{
					//g_flag_input_request = true;
					if (debug)
						printf("planning failed to send request\n");
				}
				else
				{
					send_status = 1;
					if (debug)
						printf("成功向Data Pool发送了数据请求 \n");
				}
			}
			// 接收DataPool发送的局部规划输入数据
			else if (send_status == 1)
			{
				zmq_msg_t recv_msg;
				zmq_msg_init(&recv_msg);
				zmq_msg_recv(&recv_msg, z_socket, 0);
				char *ch_recv = (char*)zmq_msg_data(&recv_msg);
				string str_recv(ch_recv);
				zmq_msg_close(&recv_msg);
				Json::Reader reader;
				Json::Value value;
				if (!reader.parse(str_recv, value, false))
				{
					//g_flag_input_request = true;
					cerr << "zmq planning json parse failed" << endl;
					cerr << "receive: " << str_recv.c_str() << endl;
				}
				else
				{
					string type = value["TYPE"].asString();

					if (type == "REP_LOCAL")
					{
						Json::Value recv_data = value["DATA"];
						Json::Value recv_sense = recv_data["SENSE"];
						Json::Value recv_GPS = recv_data["GPS"];

						// 填规划输入结构体
						memset(&strCarStatus, 0, sizeof(strCarStatus));
						iJsonToCarStatus(recv_GPS, &strCarStatus);
						memset(&strTargetsFusion, 0, sizeof(strTargetsFusion));
						iJsonToTargetsStatus(recv_sense, &strTargetsFusion);
						Timer0.StartTimer();
						// 进行规划算法处理
						int iRet = iPlanning(&strCarStatus, &strTargetsFusion, &strPlanningConfig, &strOutTrajectory);
						Timer0.StopTimer();
						double PlanTime = Timer0.GetAlgExcTime();
						printf("决策规划过程耗时%f毫秒 \n", PlanTime);
						if (strOutTrajectory.iStatus != PLAN_NORMAL)	// 规划不正常的情况下，系数参数赋值
						{
							PlanStatus = 0;
							for (int i = 0; i < 4; i++)
							{
								strOutTrajectory.pfC[i] = 0.0;
							}
							strOutTrajectory.iCleanStatus = 0;			// 停止清扫
							strOutTrajectory.iSpoutWater = 0;			// 停止喷水
						}
						else
						{
							strOutTrajectory.iCleanStatus = 1;			// 启动清扫
						}

						if (1 == strPlanningConfig.iRecordFile)
						{
							// 写入本车状态
                            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
                            auto duration_now = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
                            int ms = duration_now.count() % 1000;

                            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                            std::stringstream ss;
                            ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S.") << ms;

                            fprintf(pFd0, "%s\n", ss.str().c_str());

							// XYZ坐标为ENU东北天坐标系下的坐标
							fprintf(pFd0, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", strCarStatus.dLatitude, strCarStatus.dLongitude, strCarStatus.dAltitude, strCarStatus.pitch, strCarStatus.roll, strCarStatus.yaw, strCarStatus.fX, strCarStatus.fY, strCarStatus.fZ);
							// 写入障碍物信息
							fprintf(pFd0, "%d\n", strTargetsFusion.iTargetNum);
							for (int i = 0; i < strTargetsFusion.iTargetNum; i++)
							{
								fprintf(pFd0, "%f, %f\t", strTargetsFusion.strTargetStatus[i].fX, strTargetsFusion.strTargetStatus[i].fY);
							}
							fprintf(pFd0, "\n");

							// 写入局部参考线GPS信息iId
							fprintf(pFd0, "%d\n", iId);

							for (int i = 0; i < 50; i++)
							{
								fprintf(pFd0, "%lf\t%lf\t%lf\t%lf\t%lf\n", ReferencePoint[i].Lat, ReferencePoint[i].Lon, ReferencePoint[i].Alt, ReferencePoint[i].dX, ReferencePoint[i].dY);
							}
							fprintf(pFd0, "\n");
                            fprintf(pFd1, "%s\n", ss.str().c_str());
							for (int i = 0; i < 50; i++)
							{
								// 以下XYZ坐标是在本车坐标系下的坐标
								fprintf(pFd1, "%lf\t%lf\n", StrReferenceLine.POINT[i].X, StrReferenceLine.POINT[i].Y);
							}

							// 写入决策结果
							fprintf(pFd1, "%s\n", Decision_Type_String[Decision].c_str());

							// 写入输出点信息
							fprintf(pFd1, "%d\n", HisPlanPoints.iNum);

							// HisPlanPoints.pSLPoint中的实际数量可能是HisPlanPoints.iNum+3
							int n = 0;
							for (n = 0; n < MAX_SLPOINT_NUM; n++)
							{
								fprintf(pFd1, "%f\t%f\t", HisPlanPoints.pSLPoint[n].fX, HisPlanPoints.pSLPoint[n].fY);
							}
							fprintf(pFd1, "\n");
							// iID指的是什么?
							fprintf(pFd1, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t\%f\t%d\n", strOutTrajectory.pfC[0], strOutTrajectory.pfC[1], strOutTrajectory.pfC[2], strOutTrajectory.pfC[3], TargetPoint.fX, TargetPoint.fY, TargetPoint.fS, TargetPoint.fL, TargetPoint.iId);
							fprintf(pFd1, "\n");
						}

						unique_lock <mutex> lock_result_update(g_mutex_result_update);
						g_flag_result_update = true;
						if (debug)
							printf("唤醒局部路径规划结果发送线程\n");
						g_cv_result_update.notify_all();
					}
					else
					{
						//g_flag_input_request = true;
						if (debug)
							printf("planning received the wrong data\n");
					}
				}
				send_status = 0;
			}
		}
		cout << "server close ..." << endl;
		zmq_close(z_socket);
		zmq_term(context);
		return 0;
	}));

	// 线程2：局部路径规划模块向DataPool发送规划结果
	threads.push_back(thread([&]()
	{
		void* context = zmq_init(1);
		void* z_socket = zmq_socket(context, ZMQ_REQ);
		zmq_connect(z_socket, "tcp://127.0.0.1:5557");
		int send_status = 0;
		int send_num = 0;

		while (true)
		{
			// 向DataPool发送局部路径规划结果
			if (send_status == 0)
			{
				unique_lock <mutex> lock_result_update(g_mutex_result_update);
				while (!g_flag_result_update)
				{
					if (debug)
						printf("等待唤醒用于发送规划结果的线程 \n");
					g_cv_result_update.wait(lock_result_update);
				}
				g_flag_result_update = false;
				Timer1.StartTimer();

				// 将局部路径规划结果打包成Json格式数据
				Json::Value PlanningFram;
				Json::Value	PlanningData;
				Json::StreamWriterBuilder writerBuilder;
				ostringstream os;
				string str_send;

				PlanningData["TRAJECTORY"] = getJsonTrajectory(&strOutTrajectory);
				PlanningData["CAR_STATUS"] = getJsonCarStatus(&strCarStatus);
				PlanningData["CONTROL_CONFIG"] = getJsonControlConfig(&strControlConfig);
				PlanningData["GLOBAL"] = getJsonReferenceLine(&StrReferenceLine);
				PlanningData["READ_PLANNING_CONFIG"] = &strPlanningConfig.ReadPlanningConfig;
				PlanningData["READ_CONTROL_CONFIG"] = &strControlConfig.ReadControlConfig;
				PlanningData["READ_HD_MAP"] = &HdMap.ReadHdMap;
				PlanningData["SEARCH_REFERENCE_LINE"] = searchReferenceLine(&strCarStatus, &HdMap);
				PlanningFram["TYPE"] = "MSG_LOCAL";
				PlanningFram["DATA"] = PlanningData;

				unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
				jsonWriter->write(PlanningFram, &os);
				str_send = os.str();

				// 将Json格式数据发送给DataPool
				int length = (int)str_send.length();
				int iRet = zmq_send(z_socket, str_send.c_str(), length, 0);
				if (iRet != length)
				{
					g_flag_result_update = true;
					if (debug)
						printf("向Data Pool发送规划数据失败 \n");
				}
				else
				{
					send_status = 1;
					if (debug)
						printf("成功向Data Pool发送了规划数据 \n");
				}
			}
			// 接收DataPool的回执信息
			else if (send_status == 1)
			{
				zmq_msg_t recv_msg;
				zmq_msg_init(&recv_msg);
				zmq_msg_recv(&recv_msg, z_socket, 0);
				char *ch_recv = (char*)zmq_msg_data(&recv_msg);
				string str_recv(ch_recv);
				zmq_msg_close(&recv_msg);
				Json::Reader reader;
				Json::Value value;
				if (!reader.parse(str_recv, value, false))
				{
					g_flag_result_update = true;
					cerr << "zmq msfl json parse failed" << endl;
					cerr << "receive: " << str_recv.c_str() << endl;
				}
				else
				{
					string type = value["TYPE"].asString();
					// 接收数据池回执信息
					if (type == "REP_OK")
					{
						if (debug)
							printf("Data Pool确认收到了规划数据 \n");
#if 0
						unique_lock <mutex> lock_input_request(g_mutex_input_request);
						g_flag_input_request = true;
						if (debug)
							printf("唤醒用于请求输入数据的线程 \n");
						g_cv_input_request.notify_one();
#endif
					}
					else
					{
						cout << "receive: " << str_recv.c_str() << endl;
						g_flag_result_update = true;
						if (debug)
							printf("planning received the wrong data\n");
					}
				}

				Timer1.StopTimer();
				double PlanTotalTime = Timer1.GetAlgExcTime();
				printf("向Data Pool发送决策规划结果耗时%lf毫秒 \n", PlanTotalTime);
				send_status = 0;
			}
		}
		cout << "server close ..." << endl;
		zmq_close(z_socket);
		zmq_term(context);
	}));

	// 线程3：与调试界面通信线程（端口号5570）
	threads.push_back(thread([&]()
	{
		void* context = zmq_init(1);
		void* z_socket = zmq_socket(context, ZMQ_REQ);
		zmq_connect(z_socket, "tcp://127.0.0.1:5570");
		int send_status = 0;
		int send_num = 0;

		while (true)
		{
			// 向Debug Tool发送局部路径规划结果
			if (send_status == 0)
			{
				unique_lock <mutex> lock_result_update(g_mutex_result_update);
				while (!g_flag_result_update)
				{
					if (debug)
						printf("等待唤醒用于发送规划结果的线程 \n");
					g_cv_result_update.wait(lock_result_update);
				}
				g_flag_result_update = false;
				Timer2.StartTimer();

				// 将局部路径规划结果打包成Json格式数据
				Json::Value PlanningFram;
				Json::Value	PlanningData;
				Json::StreamWriterBuilder writerBuilder;
				ostringstream os;
				string str_send;

				PlanningData["CAR_STATUS"] = getJsonCarStatus4Debug(&strCarStatus, &VehicleSLPoint, &strPlanningConfig);
				PlanningData["REFERENCE_LINE"] = getJsonReferenceLine4Debug(&StrReferenceLine);
				PlanningData["TARGET_POINTS"] = getJsonTargetPoints4Debug(&TargetSLPoints);
				PlanningData["DECISION_POINT"] = getJsonDecisionPoint4Debug(&DecisionPoint);
				PlanningData["DECISION_STATE"] = getJsonDecisionState4Debug(&Decision, &UltrasonicDecision, &RadarDecision);
				PlanningData["TRAJECTORY"] = getJsonTrajectory4Debug(&strOutTrajectory);
				PlanningData["CLEANING_OPERATION"] = getJsonCleaningOperation4Debug(&strOutTrajectory);
				PlanningFram["TYPE"] = "MSG_PLANNING";
				PlanningFram["DATA"] = PlanningData;

				unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
				jsonWriter->write(PlanningFram, &os);
				str_send = os.str();

				// 将Json格式数据发送给DataPool
				int length = (int)str_send.length();
				int iRet = zmq_send(z_socket, str_send.c_str(), length, 0);
				if (iRet != length)
				{
					g_flag_result_update = true;
					if (debug)
						printf("向Debug Tool发送规划数据失败 \n");
				}
				else
				{
					send_status = 1;
					if (debug)
						printf("成功向Debug Tool发送了规划数据 \n");
				}
			}
			// 接收Debug Tool的回执信息
			else if (send_status == 1)
			{
				zmq_msg_t recv_msg;
				zmq_msg_init(&recv_msg);
				zmq_msg_recv(&recv_msg, z_socket, 0);
				char *ch_recv = (char*)zmq_msg_data(&recv_msg);
				string str_recv(ch_recv);
				zmq_msg_close(&recv_msg);
				Json::Reader reader;
				Json::Value value;
				if (!reader.parse(str_recv, value, false))
				{
					g_flag_result_update = true;
					cerr << "zmq msfl json parse failed" << endl;
					cerr << "receive: " << str_recv.c_str() << endl;
				}
				else
				{
					string type = value["TYPE"].asString();
					// 接收调试界面的回执信息
					if (type == "REP_OK")
					{
						if (debug)
							printf("Debug Tool确认收到了规划数据 \n");
#if 0
						unique_lock <mutex> lock_input_request(g_mutex_input_request);
						g_flag_input_request = true;
						if (debug)
							printf("唤醒用于请求输入数据的线程 \n");
						g_cv_input_request.notify_one();
#endif
					}
					else
					{
						cout << "receive: " << str_recv.c_str() << endl;
						g_flag_result_update = true;
						if (debug)
							printf("planning received the wrong data\n");
					}
				}

				Timer2.StopTimer();
				double TotalTime = Timer2.GetAlgExcTime();
				printf("向Debug Tool发送决策规划结果耗时%lf毫秒 \n", TotalTime);
				send_status = 0;
			}
		}
		cout << "server close ..." << endl;
		zmq_close(z_socket);
		zmq_term(context);
	}));

	for (auto& thread : threads)
		thread.join();

	if (NULL != pFd0)
	{
		fclose(pFd0);
		pFd0 = NULL;
	}

	if (NULL != pFd1)
	{
		fclose(pFd1);
		pFd1 = NULL;
	}

	return 0;
}

