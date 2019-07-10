#ifndef _PLANNING_H_
#define _PLANNING_H_

#include "data_struct_define.h"
#include "route_planner.h"
#include "speed_planner.h"
#include "cost_select.h"

#ifdef __cplusplus 
extern "C"
{
#endif

	// 规划初始化接口
	int iPlanningInit(char *pchDir, StrPlanningConfig *pStrPlanningConfig);
	// 规划主函数接口
	int iPlanning(StrLocationFusion *pStrCarStatus,				// 本车状态
		StrTargetFusion *pTargetsFusion,						// 目标融合输出
		StrPlanningConfig *pStrPlanningConfig,					// 配置参数
		StrSLPoints *pStrOutTrajectory);						// 输出轨迹点
	// 规划反初始化函数
	int iDePlanningInit(StrPlanningConfig *pStrPlanningConfig);
	int iPlanningProcessInit(void);

#ifdef __cplusplus
}
#endif

#endif
