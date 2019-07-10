#ifndef _SPEED_PLANNER_H_
#define   _SPEED_PLANNER_H_

#include "data_struct_define.h"
#include "route_planner.h"

#ifdef __cplusplus 
extern "C"
{
#endif

	float fLSolver(float *pfC, float fS, int order);
	int iSpeedPlanner(StrRoutePaths *pStrRoutePaths, StrPlanningConfig *pStrPlanningConfig);
	double fThetaFit(double x1, double x2, double x3, double y1, double y2, double y3);

#ifdef __cplusplus
}
#endif

#endif
