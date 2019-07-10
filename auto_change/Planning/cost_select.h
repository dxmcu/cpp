#ifndef _COST_SELECT_H_
#define _COST_SELECT_H_

#include "data_struct_define.h"

#ifdef __cplusplus 
extern "C"
{
#endif

	int iCostSelect(StrRoutePaths * pStrTrajectorys,	// 候选轨迹集
		StrTargetFusion *pTargetsFusion,
		StrSLPoints *pStrSLReferLines,
		StrPlanningConfig *pStrPlanningConfig,			// 配置参数
		StrSLPoints *pStrOutTrajectory);				// 最优轨迹输出

#ifdef __cplusplus
}
#endif

#endif
