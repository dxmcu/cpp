#ifndef _ROUTE_PLANNER_H_
#define   _ROUTE_PLANNER_H_

#include "data_struct_define.h"
#include "speed_planner.h"

#ifdef __cplusplus 
extern "C"
{
#endif

	void vCal_l(float *pfC, int iNum, StrSLPoints *pStrSLReferLines, float *pfL, int iOutNum);
	int vCurtoCar(StrSLPoints *pStrSLReferLines, float *pfL, int iNum, StrSLPoints *RoutePoints);
	float fCalRoadWidth(StrTargetFusion *pTargetsFusion, StrPlanningConfig *pStrPlanningConfig);
	int iReferLineXYToSL(Global *pStrXYReferLine, StrLocationFusion *pStrCarStatus, StrSLPoints *pStrSLReferLine);
	int iFindNearestPoint(SLPoint *pVehicleStatus, StrSLPoints *pStrSLReferLines);
	void polyfit(int n, double x[], double y[], int poly_n, double p[]);
	int iFindHisPlanId(StrLocationFusion *pStrCarStatus, MapPoint *ReferencePoint);
	int iRoutePlanner(StrSLPoints *pStrSLReferLines,	// 参考路径
		StrLocationFusion *pStrCarStatus,				// 本车状态
		StrTargetFusion *pTargetsFusion,				// 目标融合结果
		StrPlanningConfig *pStrPlanningConfig,			// 配置参数
		StrRoutePaths *pStrLocalPaths);					// Output

#ifdef __cplusplus
}
#endif

#endif
