#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <map>
#include "cost_select.h"
#include "route_planner.h"

extern bool debug;

double softmax(float x, float x0)
{
	return exp(-(x - x0)) / (1.0 + exp(-(x - x0)));
}

double quasi_softmax(float x, float l0, float b, float k)
{
	return (b + exp(-k * (x - l0))) / (1.0 + exp(-k * (x - l0)));
}

SLPoint XYtoSL(SLPoint XYPoint, StrSLPoints *pStrSLReferLines)
{
	SLPoint ReturnPoint;
	int iId;

	iId = iFindNearestPoint(&XYPoint, pStrSLReferLines);
	ReturnPoint.fS = pStrSLReferLines->pSLPoint[iId].fS;
	ReturnPoint.fL = sqrt((XYPoint.fX - pStrSLReferLines->pSLPoint[iId].fX)*(XYPoint.fX - pStrSLReferLines->pSLPoint[iId].fX) 
		+ (XYPoint.fY - pStrSLReferLines->pSLPoint[iId].fY)*(XYPoint.fY - pStrSLReferLines->pSLPoint[iId].fY));

	return ReturnPoint;
}

int DrawObstalces(StrTargetStatus *pObstalceTarget, StrSLPoints *pStrSLReferLines, StrSLBoundary *pBoundary)
{
	int i = 0;
	SLPoint pCorners[EDGE_NUM];
	SLPoint pSLCorners[EDGE_NUM];

	for (i = 0; i < EDGE_NUM; i++)
	{
		pCorners[i].fX = pObstalceTarget->strTargetBoard.strTargetPoint[i].fX;
		pCorners[i].fY = pObstalceTarget->strTargetBoard.strTargetPoint[i].fY;
		pCorners[i].fZ = pObstalceTarget->strTargetBoard.strTargetPoint[i].fZ;
		pSLCorners[i] = XYtoSL(pCorners[i], pStrSLReferLines);
	}

	pBoundary->fMaxS = -FLT_MAX;	// maxS
	pBoundary->fMinS = FLT_MAX;		// minS
	pBoundary->fMaxL = -FLT_MAX;	// maxL
	pBoundary->fMinL = FLT_MAX;		// minL

	for (i = 0; i < EDGE_NUM; i++)
	{
		if (pSLCorners[i].fS > pBoundary->fMaxS)
			pBoundary->fMaxS = pSLCorners[i].fS;

		if (pSLCorners[i].fS < pBoundary->fMinS)
			pBoundary->fMinS = pSLCorners[i].fS;

		if (pSLCorners[i].fL > pBoundary->fMaxL)
			pBoundary->fMaxL = pSLCorners[i].fL;

		if (pSLCorners[i].fL < pBoundary->fMinL)
			pBoundary->fMinL = pSLCorners[i].fL;
	}

	return 0;
}

// 道路边界检测
float fRoadBoundaryCheck(StrSLPoints *pRoutePath, StrTargetFusion *pTargetsFusion, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int j = 0;
	float fLaneWidth = 0.0;
	float fTolerance = 0.0;

	StrSLPoints RoutePath;
	float fSum = 0.0;
	float fCost = 0.0;

	if (NULL == pRoutePath || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	for (i = 0; i < pRoutePath->iNum; i++)
	{
		fLaneWidth = fCalRoadWidth(pTargetsFusion, pStrPlanningConfig);
		fTolerance = (float)(fLaneWidth - pStrPlanningConfig->fVehicleWidth - pStrPlanningConfig->fSampleBuffer);
		if (fabs(RoutePath.pSLPoint[0].fL) * 2 > fTolerance)
			fTolerance = fabs(RoutePath.pSLPoint[0].fL) * 2 + 0.3;

		if (fabs(RoutePath.pSLPoint[i].fL) > (fTolerance / 2.0))
			fSum += 1.0;
		else
			fSum += 0.0;
	}

	if (fSum > 0)
		fCost = FLT_MAX;
	else
		fCost = 0.0;

	return fCost;
}

float CheckOverlap(StrSLPoints *pRoutePath, StrTargetStatus *pTargetStatus, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;

	float fOverlap = 0.0;
	StrSLBoundary Boundary;
	float fVMaxL = 0.0;
	float fVMinL = 0.0;
	float fVMaxS = 0.0;
	float fVMinS = 0.0;
	DrawObstalces(pTargetStatus, pStrSLReferLines, &Boundary);
	float fMaxS = Boundary.fMaxS;
	float fMinS = Boundary.fMinS;
	float fMaxL = Boundary.fMaxL;
	float fMinL = Boundary.fMinL;
	for (i = 0; i < pRoutePath->iNum; i++)
	{
		// 车身长度的最大值和最小值没有问题，但是车身宽度的最大值和最小值是否需要调整？因为我们的扫地车是一个铰接模型。
		fVMaxL = pRoutePath->pSLPoint[i].fL + pStrPlanningConfig->fVehicleWidth / 2.0;
		fVMinL = pRoutePath->pSLPoint[i].fL - pStrPlanningConfig->fVehicleWidth / 2.0;
		fVMaxS = pRoutePath->pSLPoint[i].fS + pStrPlanningConfig->fVehicleLength / 2.0;
		fVMinS = pRoutePath->pSLPoint[i].fS - pStrPlanningConfig->fVehicleLength / 2.0;
		if (((fVMaxS < fMinS) || (fVMinS > fMaxS)) || (((fVMinL - pStrPlanningConfig->fObstacleBuffer) > fMaxL) || ((fVMaxL + pStrPlanningConfig->fObstacleBuffer) < fMinL)))
			fOverlap = FLT_MAX;
		else
		{
			fOverlap = fMinS;
			return fOverlap;
		}
	}

	return fOverlap;
}

float fObstacleCheck(StrSLPoints *pRoutePath, StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	float fCost = 0.0;
	float fOverlap = 0.0;
	float OverlapBoard = 0.0;

	for (i = 0; i < pTargetsFusion->iTargetNum; i++)
	{
		fOverlap = CheckOverlap(pRoutePath, &pTargetsFusion->strTargetStatus[i], pStrSLReferLines, pStrPlanningConfig);
		fCost += fOverlap;
	}

	return fCost;
}

float fSmoothnessCost(StrSLPoints *pRoutePath, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	float fCost = 0.0;
	float fl0 = (float)pStrPlanningConfig->fl0;
	float fb = (float)pStrPlanningConfig->fb;
	float fk = (float)pStrPlanningConfig->fk;
	float fL = 0.0;
	float fDL = 0.0;
	float fDDL = 0.0;

	for (i = 0; i < pRoutePath->iNum; i++)
	{
		fL = pRoutePath->pSLPoint[i].fL;
		fDL = pRoutePath->pSLPoint[i].fDL;
		fDDL = pRoutePath->pSLPoint[i].fDDL;
		fCost += fL * fL * pStrPlanningConfig->fPathLCost * quasi_softmax(fabs(fL), fl0, fb, fk);
		fCost += fDL * fDL * pStrPlanningConfig->fPathDLCost;
		fCost += fDDL * fDDL * pStrPlanningConfig->fPathDDLCost;
	}
	fCost *= pStrPlanningConfig->fUnitS;
	fCost += (pStrPlanningConfig->fEndCost * pRoutePath->pSLPoint[pRoutePath->iNum - 1].fL * pRoutePath->pSLPoint[pRoutePath->iNum - 1].fL);

	return fCost;
}

float fObstacleCost(StrSLPoints *pRoutePath, StrTargetFusion *pObstacles, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int j = 0;
	float fCost = 0.0;
	float fVMaxL = 0.0;
	float fVMinL = 0.0;
	float fVMaxS = 0.0;
	float fVMinS = 0.0;
	float fDeltaL = 0.0;
	float fDeltaS = 0.0;
	StrSLBoundary Boundary;

	for (i = 0; i < pObstacles->iTargetNum; i++)
	{
		DrawObstalces(&pObstacles->strTargetStatus[i], pStrSLReferLines, &Boundary);
		if ((Boundary.fMinS > pRoutePath->pSLPoint[pRoutePath->iNum - 1].fS) || (Boundary.fMaxS < pRoutePath->pSLPoint[0].fS))
			continue;
		else
		{
			for (j = 0; j < pRoutePath->iNum; j++)
			{
				fVMaxL = pRoutePath->pSLPoint[j].fL + pStrPlanningConfig->fVehicleWidth / 2.0;
				fVMinL = pRoutePath->pSLPoint[j].fL - pStrPlanningConfig->fVehicleWidth / 2.0;
				fVMaxS = pRoutePath->pSLPoint[j].fS + pStrPlanningConfig->fVehicleLength / 2.0;
				fVMinS = pRoutePath->pSLPoint[j].fS - pStrPlanningConfig->fVehicleLength / 2.0;
				fDeltaL = fabs(pRoutePath->pSLPoint[j].fL - (Boundary.fMaxL + Boundary.fMinL) / 2.0);
				fDeltaS = fabs(pRoutePath->pSLPoint[j].fS - (Boundary.fMaxS + Boundary.fMinS) / 2.0);
				if ((fDeltaS < pStrPlanningConfig->fMaxPlanningDistance) && (fmin(fabs(Boundary.fMaxL), fabs(Boundary.fMinL)) < pStrPlanningConfig->fIgnoreL) 
					&& (pObstacles->strTargetStatus[i].emMotionStatus == 0))
					fCost += ((pStrPlanningConfig->fMaxPlanningDistance - fDeltaS) / pStrPlanningConfig->fMaxPlanningDistance) * ((pStrPlanningConfig->fMaxPlanningDistance - fDeltaS) / pStrPlanningConfig->fMaxPlanningDistance) *
					pStrPlanningConfig->fObstacleCollisionCost * (softmax(fDeltaL, pStrPlanningConfig->fObstacleCollisionDis) + softmax(fDeltaS, pStrPlanningConfig->fObstacleCollisionDis));
			}
		}
	}
	fCost *= pStrPlanningConfig->fUnitS;

	return fCost;
}

int iChoosePath(StrRoutePaths *pStrTrajectorys, StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pStrOutTrajectory)
{
	int i = 0;
	int j = 0;
	float pfObstacleCost[MAX_SLPOINT_NUM];
	float pfSmoothCost[MAX_SLPOINT_NUM];
	StrSLPoints Path;

	if ((NULL == pStrTrajectorys) || (NULL == pTargetsFusion) || (NULL == pStrSLReferLines) || (NULL == pStrPlanningConfig))
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (NULL == pStrOutTrajectory)
	{
		if (debug)
			printf("Output Param Error\n");
		return -1;
	}

	for (i = 0; i < pStrTrajectorys->iRouteNum; i++)
	{
		Path = pStrTrajectorys->strRoutePath[i];
		pfObstacleCost[i] = fObstacleCost(&Path, pTargetsFusion, pStrSLReferLines, pStrPlanningConfig);
		pfSmoothCost[i] = fSmoothnessCost(&Path, pStrPlanningConfig);
		pStrTrajectorys->fCost[i] += pfObstacleCost[i] + pfSmoothCost[i];
	}

	float fMinCost = pStrTrajectorys->fCost[0];
	int iIndex = 0;
	// index是否应该从1开始？
	for (i = 1; i < pStrTrajectorys->iRouteNum; i++)
	{
		if (pStrTrajectorys->fCost[i] < fMinCost)
		{
			iIndex = i;
			fMinCost = pStrTrajectorys->fCost[i];
		}
	}

	*pStrOutTrajectory = pStrTrajectorys->strRoutePath[iIndex];

	return 0;
}

int iCostSelect(StrRoutePaths * pStrTrajectorys, StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pStrOutTrajectory)
{
	int iRet = 0;
	int i = 0;
	float fBoundaryCost = 0.0;
	float fObstacleCost = 0.0;
	int iCandidateRouteNum = 0;
	pStrOutTrajectory->iStatus = PLAN_NORMAL;
	// 只规划了一条路径
	if (1 == pStrTrajectorys->iRouteNum)
	{
		memcpy(pStrOutTrajectory, &pStrTrajectorys->strRoutePath[0], sizeof(StrSLPoints));
		return 0;
	}

	for (i = 0; i < pStrTrajectorys->iRouteNum; i++)
	{
		// 检测道路边界
		fBoundaryCost = fRoadBoundaryCheck(&pStrTrajectorys->strRoutePath[i], pTargetsFusion, pStrPlanningConfig);
		pStrTrajectorys->fCost[i] = fBoundaryCost;
	}

	for (i = 0; i < pStrTrajectorys->iRouteNum; i++)
	{
		// 检测障碍物
		fObstacleCost = fObstacleCheck(&pStrTrajectorys->strRoutePath[i], pTargetsFusion, pStrSLReferLines, pStrPlanningConfig);
		pStrTrajectorys->fCost[i] += fObstacleCost;
		if (pStrTrajectorys->fCost[i] < FLT_MAX)
		{
			iCandidateRouteNum++;
		}
	}

	if (0 == iCandidateRouteNum)
	{
		if (debug)
			printf("所有轨迹都不可用\n");
		for (i = 0; i < pStrOutTrajectory->iNum; i++)
		{
			pStrOutTrajectory->pSLPoint[i].iId = i;
			pStrOutTrajectory->pSLPoint[i].fX = pStrOutTrajectory->pSLPoint[0].fX;
			pStrOutTrajectory->pSLPoint[i].fY = pStrOutTrajectory->pSLPoint[0].fY;
			pStrOutTrajectory->pSLPoint[i].fZ = pStrOutTrajectory->pSLPoint[0].fZ;
			pStrOutTrajectory->pSLPoint[i].fTheta = pStrOutTrajectory->pSLPoint[0].fTheta;
			pStrOutTrajectory->pSLPoint[i].fKappa = pStrOutTrajectory->pSLPoint[0].fKappa;
			pStrOutTrajectory->pSLPoint[i].fV = 0.0;
			pStrOutTrajectory->pSLPoint[i].fA = -pStrPlanningConfig->fMaxAcc;
		}

		return 0;
	}

	iRet = iChoosePath(pStrTrajectorys, pTargetsFusion, pStrSLReferLines, pStrPlanningConfig, pStrOutTrajectory);
	if (-1 == iRet)
	{
		if (debug)
			printf(" ChoosePath Error\n");
		return -1;
	}

	return 0;
}

