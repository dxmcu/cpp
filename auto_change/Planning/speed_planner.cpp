#include <stdio.h>
#include <string.h>
#include <math.h>
#include "speed_planner.h"

extern bool debug;

// 求解L，LD，LDD
float fLSolver(float *pfC, float fS, int order)
{
	if (order == 0)
		return pfC[0] + pfC[1] * fS + pfC[2] * fS * fS + pfC[3] * fS * fS * fS;
	else if (order == 1)
		return pfC[1] + 2 * pfC[2] * fS + 3 * pfC[3] * fS * fS;
	else if (order == 2)
		return 2 * pfC[2] + 6 * pfC[3] * fS;
	else if (order == 3)
		return 6 * pfC[3];
	else
		return 0.0;
}

// 计算每个规划轨迹点的S坐标值：S_j - S_{j-1} = V_{j-1} * delta_t + 0.5 * a_{j-1} * delta_t^2
// 虽然在进行规划时为每个轨迹点的S坐标赋值了，但是，到了这里要重新计算每个轨迹点的S坐标值
// 根据三阶曲线函数L=f(S)，进一步计算每个规划轨迹点的L坐标、DL坐标、DDL坐标
// 把计算结果存放在规划结果pStrRoutePaths中
int iCalS(StrRoutePaths *pStrRoutePaths, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int j = 0;
	int iNum = 0;
	float fPlanTime = 0.0;
	float pfC[4];
	int iNumfC = 4;
	float pfL[MAX_SLPOINT_NUM];

	if (NULL == pStrRoutePaths || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (pStrRoutePaths->iRouteNum < 1)
	{
		if (debug)
			printf("Route Paths Num Error\n");
		return -1;
	}

	fPlanTime = (float)pStrPlanningConfig->fSampleTime;

	for (i = 0; i < pStrRoutePaths->iRouteNum; i++)
	{
		iNum = pStrRoutePaths->strRoutePath[i].iNum;
		// 第一个轨迹点的S坐标没有被更新，可能不太重要
		// S_j - S_{j-1} = V_{j-1} * delta_t + 0.5 * a_{j-1} * delta_t^2
		for (j = 1; j < iNum; j++)
		{
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fS = pStrRoutePaths->strRoutePath[i].pSLPoint[j - 1].fS
				+ pStrRoutePaths->strRoutePath[i].pSLPoint[j - 1].fV * fPlanTime + 0.5 * pStrRoutePaths->strRoutePath[i].pSLPoint[j - 1].fA * fPlanTime * fPlanTime;
		}
	}

	for (i = 0; i < pStrRoutePaths->iRouteNum; i++)
	{
		iNum = pStrRoutePaths->strRoutePath[i].iNum;
		memcpy(pfC, pStrRoutePaths->strRoutePath[i].pfC, 4 * sizeof(float));
		for (j = 1; j < iNum; j++)
		{
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fL = fLSolver(pfC, pStrRoutePaths->strRoutePath[i].pSLPoint[j].fS, 0);
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fDL = fLSolver(pfC, pStrRoutePaths->strRoutePath[i].pSLPoint[j].fS, 1);
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fDDL = fLSolver(pfC, pStrRoutePaths->strRoutePath[i].pSLPoint[j].fS, 2);
		}
	}

	return 0;
}

// 利用轨迹序列的起始点和终点计算规划轨迹点的加速度，a = {(v_e)^2 - (v_s)^2} / (2 * delta_s)
// 加速度值的约束区间是:[-3.0, 3.0]，单位是：m/{s^2}
// 注意：每个规划轨迹点的加速度都是相同的，程序假设了车辆是匀加速运动的
// 把计算结果存放在规划结果pStrRoutePaths中
int iCalAcc(StrRoutePaths *pStrRoutePaths, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int j = 0;
	float fSEnd = 0.0;
	float fSStart = 0.0;
	float fVStart = 0.0;
	float fVEnd = 0.0;
	int iNum = 0;
	float fAcc = 0.0;

	if (NULL == pStrRoutePaths || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (pStrRoutePaths->iRouteNum < 1)
	{
		if (debug)
			printf("Route Paths Num Error\n");
		return -1;
	}

	for (i = 0; i < pStrRoutePaths->iRouteNum; i++)
	{
		iNum = pStrRoutePaths->strRoutePath[i].iNum;
		fSStart = pStrRoutePaths->strRoutePath[i].pSLPoint[0].fS;
		fSEnd = pStrRoutePaths->strRoutePath[i].pSLPoint[iNum - 1].fS;
		fVStart = pStrRoutePaths->strRoutePath[i].pSLPoint[0].fV;
		fVEnd = pStrRoutePaths->strRoutePath[i].pSLPoint[iNum - 1].fV;
		fAcc = (fVEnd * fVEnd - fVStart * fVStart) / (2 * (fSEnd - fSStart));
		if (fAcc > pStrPlanningConfig->fMaxAcc)
			fAcc = pStrPlanningConfig->fMaxAcc;
		if (fAcc < -pStrPlanningConfig->fMaxAcc)
			fAcc = -pStrPlanningConfig->fMaxAcc;
		for (j = 0; j < iNum; j++)
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fA = fAcc;
	}

	return 0;
}

// 使用加速度计算每个规划轨迹点的速度: v_j = v_{j-1} + a_{j-1} * delta_t
// 最大速度为：2 m/s
// 把计算结果存放在规划结果pStrRoutePaths中
int iCalSpeed(StrRoutePaths *pStrRoutePaths, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int j = 0;
	float fSEnd = 0.0;
	float fSStart = 0.0;
	float fVStart = 0.0;
	float fVEnd = 0.0;
	int iNum = 0;
	// 从规划配置文件中读取的"采样时间"，值等于0.3秒
	// 为什么取值为0.3秒，可能的原因是在制作地图时，每个参考点的采样时间是0.3秒
	float fPlanTime = (float)pStrPlanningConfig->fSampleTime;

	if (NULL == pStrRoutePaths || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (pStrRoutePaths->iRouteNum < 1)
	{
		if (debug)
			printf("pStrRoutePaths Num Error\n");
		return -1;
	}

	for (i = 0; i < pStrRoutePaths->iRouteNum; i++)
	{
		iNum = pStrRoutePaths->strRoutePath[i].iNum;
		for (j = 1; j < iNum; j++)
		{
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fV = pStrRoutePaths->strRoutePath[i].pSLPoint[j - 1].fV + pStrRoutePaths->strRoutePath[i].pSLPoint[j - 1].fA * fPlanTime;
			if (pStrRoutePaths->strRoutePath[i].pSLPoint[j].fV > pStrPlanningConfig->fMaxV)
				pStrRoutePaths->strRoutePath[i].pSLPoint[j].fV = pStrPlanningConfig->fMaxV;
		}
	}

	return 0;
}

// 最小二乘法拟合航向
// 返回值fTheta的理论取值范围为:(-180, 180)度
double fThetaFit(double x1, double x2, double x3, double y1, double y2, double y3)
{
	double fTheta1;
	double fTheta2;
	double deltx1 = x2 - x1;
	double delty1 = y2 - y1;
	double deltr1 = sqrt(deltx1 * deltx1 + delty1 * delty1);
	double deltx2 = x3 - x2;
	double delty2 = y3 - y2;
	double deltr2 = sqrt(deltx2 * deltx2 + delty2 * delty2);
	double fcos1 = deltx1 / deltr1;
	double fsin1 = delty1 / deltr1;
	double fcos2 = deltx2 / deltr2;
	double fsin2 = delty2 / deltr2;
	// 车辆方向盘的转向角度范围为：左右各60度
	if (fsin1 >= 0)		// 车辆往左转向，theta角度在[0,90]区间
		fTheta1 = acos(fcos1);
	else				// 车辆往右转向，theta角度在[-90,0]
		fTheta1 = -acos(fcos1);

	if (fsin2 >= 0)		// 车辆往左转向，theta角度在[0,90]区间
		fTheta2 = acos(fcos2);
	else				// 车辆往右转向，theta角度在[-90,0]
		fTheta2 = -acos(fcos2);
	// 取两次转向角度的平均值(第一次：1号点->2号点, 第二次：2号点->3号点)
	double fTheta = (fTheta1 + fTheta2) / 2;

	return fTheta;
}

// 利用当前轨迹点的上一个轨迹点和下一个轨迹点，计算每个轨迹点的theta角度
// theta角度表示车体坐标系X轴正方向与在当前轨迹点的车运动方向之间的夹角
int iThetaFilter(StrSLPoints *pStrRoutePath)
{
	int i = 0;
	float ftheta = 0.0;
	SLPoint Point1;
	SLPoint Point2;
	SLPoint Point;
	int iPointsNum = 0;

	if (NULL == pStrRoutePath)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	iPointsNum = pStrRoutePath->iNum;
	for (i = 1; i < iPointsNum - 1; i++)
	{
		Point1 = pStrRoutePath->pSLPoint[i + 1];		// 下一个轨迹点
		Point2 = pStrRoutePath->pSLPoint[i - 1];		// 上一个轨迹点
		Point = pStrRoutePath->pSLPoint[i];

		// ftheta表示车在当前轨迹点的运动方向与车体坐标系的X轴之间的夹角
		// 上一个轨迹点和下一个轨迹点具有相同的X坐标
		if (fabs(Point2.fX - Point1.fX) < 0.0001)
		{
			// 以下的这个if else判断条件，可能存在一个小问题
			// 下一个轨迹点的Y坐标大于上一个轨迹点的Y坐标
			if (Point1.fY > Point2.fY)
				ftheta = 0.5 * PI;
			else if (fabs(Point1.fY - Point2.fY) < 0.0001)
				ftheta = Point2.fTheta;
			// 下一个轨迹点的Y坐标小于上一个轨迹点的Y坐标
			else
				ftheta = -0.5 * PI;
		}
		else
		{
			// 上一个轨迹点和下一个轨迹点具有相同的Y坐标
			if (fabs(Point1.fY - Point2.fY) <= 0.0001)
			{
				// 上一个轨迹点的Y坐标小于下一个轨迹点的Y坐标
				if (Point2.fX < Point1.fX)
					ftheta = 0.0;
				// 上一个轨迹点的Y坐标大于下一个轨迹点的Y坐标
				else
					ftheta = PI;
			}
			else
			{
				// 上一个轨迹点的X坐标小于下一个轨迹点的X坐标
				if (Point2.fX < Point1.fX)
					ftheta = fThetaFit(Point2.fX, Point.fX, Point1.fX, Point2.fY, Point.fY, Point1.fY);			// theta角度的取值范围是:(-90, 90)
				// 上一个轨迹点的X坐标大于下一个轨迹点的X坐标，而且上一个轨迹点的Y坐标小于下一个轨迹点的Y坐标
				else if ((Point2.fX > Point1.fX) && (Point2.fY < Point1.fY))
					ftheta = fThetaFit(Point2.fX, Point.fX, Point1.fX, Point2.fY, Point.fY, Point1.fY) + PI;	// ftheta角度的理论取值范围是：(-180, 0) + 180 = (0, 180)
				// 上一个轨迹点的X坐标大于下一个轨迹点的X坐标，而且上一个轨迹点的Y坐标大于下一个轨迹点的Y坐标
				else
					ftheta = fThetaFit(Point2.fX, Point.fX, Point1.fX, Point2.fY, Point.fY, Point1.fY) - PI;	// ftheta角度的理论取值范围是：(0, 180) - 180 = (-180, 0)，也等于(180, 360)
			}
		}
		pStrRoutePath->pSLPoint[i].fTheta = ftheta;
	}

	pStrRoutePath->pSLPoint[0].fTheta = pStrRoutePath->pSLPoint[1].fTheta;
	pStrRoutePath->pSLPoint[iPointsNum - 1].fTheta = pStrRoutePath->pSLPoint[iPointsNum - 2].fTheta;

	return 0;
}

// Kappa是转弯半径的倒数
int iKbCircle(SLPoint *pPrePoint, SLPoint *pCurPoint, SLPoint *pSucPoint)
{
	float fRawKappa = 0.0;
	float x1 = pPrePoint->fX;
	float x2 = pCurPoint->fX;
	float x3 = pSucPoint->fX;
	float y1 = pPrePoint->fY;
	float y2 = pCurPoint->fY;
	float y3 = pSucPoint->fY;

	// 前一个点1、本点2、后一个点3：这三个点在同一条线上
	if ((fabs(((x3 - x1)*(y2 - y1) - (x2 - x1)*(y3 - y1))) < EQUAL) || (fabs(((y3 - y1)*(x2 - x1) - (y2 - y1)*(x3 - x1))) < EQUAL))
		fRawKappa = 0.0;			// 车辆直行：转弯半径的倒数等于0，即转弯半径无穷大
	else
	{
		float a = ((y2 - y1)*(y3*y3 - y1*y1 + x3*x3 - x1*x1) - (y3 - y1)*(y2*y2 - y1*y1 + x2*x2 - x1*x1)) / (2.0*((x3 - x1)*(y2 - y1) - (x2 - x1)*(y3 - y1)));
		float b = ((x2 - x1)*(x3*x3 - x1*x1 + y3*y3 - y1*y1) - (x3 - x1)*(x2*x2 - x1*x1 + y2*y2 - y1*y1)) / (2.0*((y3 - y1)*(x2 - x1) - (y2 - y1)*(x3 - x1)));
		float r = sqrt((x1 - a)*(x1 - a) + (y1 - b)*(y1 - b));		// 转弯半径

		// 转弯半径的最小值为MINR(6.75米)
		if (r < MINR)
			r = MINR;
		fRawKappa = (float)1.0 / r;
		// 转弯半径的倒数的最大值为MAXKAPPA（0.1538），相应地，转弯半径的最小值为6.5米
		if (fRawKappa > MAXKAPPA)	// limitation of kappa. should not be bigger than the max kappa of the vechicle.
			fRawKappa = MAXKAPPA;
		if (fRawKappa < -MAXKAPPA)	// limitation of kappa. should not be bigger than the max kappa of the vechicle.
			fRawKappa = -MAXKAPPA;
	}

	// fTheta角度表示车体坐标系X轴正方向与在当前轨迹点的车运动方向之间的夹角，理论的取值区间为：[-PI, PI]
	// X轴朝着车辆的向前方向，Y轴朝着车辆的左侧方向
	// 转弯半径的倒数kappa的符号：正表示往右转弯，负表示往左转弯
	if (fabs(pPrePoint->fTheta - pSucPoint->fTheta) < 1.8 * PI)		// 大部分情况落在这个条件下
	{
		// 从上一个轨迹点到下一个轨迹点，车辆的运动方向往右偏了
		if (pPrePoint->fTheta > pSucPoint->fTheta)
			pCurPoint->fKappa = fRawKappa;		// 正值表示往右转弯
		// 从上一个轨迹点到下一个轨迹点，车辆的运动方向往左偏了
		else if (pPrePoint->fTheta < pSucPoint->fTheta)
			pCurPoint->fKappa = -fRawKappa;		// 负值表示往左转弯
		// 从上一个轨迹点到下一个轨迹点，车辆的运动方向保持不变
		else
			pCurPoint->fKappa = 0;				// 0表示直行
	}
	else
	{
		if (pPrePoint->fTheta > pSucPoint->fTheta)
			pCurPoint->fKappa = -fRawKappa;
		else if (pPrePoint->fTheta < pSucPoint->fTheta)
			pCurPoint->fKappa = fRawKappa;
		else
			pCurPoint->fKappa = 0;
	}

	return 0;
}

// 利用当前轨迹点的上一个轨迹点和下一个轨迹点的XY坐标,计算每个轨迹点的转弯半径的倒数Kappa
int iKbFilter(StrSLPoints *pStrRoutePath)
{
	int i = 0;
	int j = 0;
	int iPointsNum = 0;

	if (NULL == pStrRoutePath)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	iPointsNum = pStrRoutePath->iNum;
	for (i = 1; i < iPointsNum - 1; i++)
		iKbCircle(&pStrRoutePath->pSLPoint[i - 1], &pStrRoutePath->pSLPoint[i], &pStrRoutePath->pSLPoint[i + 1]);

	pStrRoutePath->pSLPoint[0].fKappa = 0.0;					// 直行
	pStrRoutePath->pSLPoint[iPointsNum - 1].fKappa = 0.0;		// 直行

	return 0;
}

int iSpeedPlanner(StrRoutePaths *pStrRoutePaths, StrPlanningConfig *pStrPlanningConfig)
{
	int i = 0;
	int iRet = 0;
	if (debug)
		printf("enter the Speed planner\n");

	if (NULL == pStrRoutePaths || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	// 利用轨迹点序列的起始点和终点计算规划轨迹点的加速度，每个规划轨迹点的加速度都是相同的，程序假设车辆是匀加速运动的
	// 把计算结果存放在规划结果pStrRoutePaths中
	iRet = iCalAcc(pStrRoutePaths, pStrPlanningConfig);
	if (-1 == iRet)
	{
		if (debug)
			printf("iCalAcc Error\n");
		return -1;
	}

	// 使用已经计算好的加速度值计算每个规划轨迹点的速度: v_j = v_{j-1} + a_{j-1} * delta_t
	// 把计算结果存放在规划结果pStrRoutePaths中
	iRet = iCalSpeed(pStrRoutePaths, pStrPlanningConfig);
	if (-1 == iRet)
	{
		if (debug)
			printf("iCalSpeed Error\n");
		return -1;
	}

	// 计算每个规划轨迹点的S坐标值，根据三阶曲线函数L=f(S)，进一步计算每个规划轨迹点的L坐标、DL坐标、DDL坐标
	// 把计算结果存放在规划结果pStrRoutePaths中
	iRet = iCalS(pStrRoutePaths, pStrPlanningConfig);
	if (-1 == iRet)
	{
		if (debug)
			printf("Cal S Error\n");
		return -1;
	}

	for (i = 0; i < pStrRoutePaths->iRouteNum; i++)
	{
		// 利用当前轨迹点的上一个轨迹点和下一个轨迹点，计算每个轨迹点的theta角度(车体坐标系X轴正方向与在当前轨迹点的车运动方向之间的夹角)
		iThetaFilter(&pStrRoutePaths->strRoutePath[i]);
		// 利用当前轨迹点的上一个轨迹点和下一个轨迹点的XY坐标,计算每个轨迹点的转弯半径的倒数Kappa
		iKbFilter(&pStrRoutePaths->strRoutePath[i]);
	}

	return 0;
}


