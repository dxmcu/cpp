#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
//#include <direct.h>
//#include <windows.h>
#include <map>
#include <string>
#include <unistd.h>
#include "route_planner.h"
#include "speed_planner.h"

using namespace std;

map<int, string> Sensor_Type_String = { { 0, "UNKNOWN" }, { 1, "RADAR" }, { 2, "LIDAR" }, { 3, "CAMERA" }, { 4, "ULTRASONIC" } };
extern SLPoint TargetPoint;
extern SLPoint VehicleSLPoint;
extern SLPoint DecisionPoint;
extern EmDecision UltrasonicDecision;
extern EmDecision RadarDecision;
extern EmDecision Decision;
extern EmDecision HisDecision;				// 每一帧决策之后，保存当前帧的决策，用于辅助下一帧的决策
extern int PlanStatus;
extern StrSLPoints HisPlanPoints;
extern MapPoint HisReferencePoint[50];
extern StrSLPoints HisStrSLReferLine;
extern SLPoint PlanStartPoint;
extern int HisId;
extern bool debug;
extern StrSLPoints TargetSLPoints;
extern int LeftTargetNum;
extern int RightTargetNum;

float TargetDeviationDis = 0.0;				// 目标偏移距离，只要障碍物有一部分压在参考线上，TargetDeviationDis就等于障碍物宽度的一半与障碍物的L坐标值之间的差值；否则，TargetDeviationDis等于0）
SLPoint UltrasonicDecisionPoint;
int SafeStopNum = 0;
EmDecision SafestopDecision;
#define N 4

// 按第一行展开计算|A|
double getA(double arcs[N][N], int n)
{
	if (n == 1)
		return arcs[0][0];

	double ans = 0;
	double temp[N][N] = { 0.0 };
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];
		}
		double t = getA(temp, n - 1);
		if (i % 2 == 0)
			ans += arcs[0][i] * t;
		else
			ans -= arcs[0][i] * t;
	}

	return ans;
}

// 计算每一行每一列的每个元素所对应的余子式，组成A*
void  getAStart(double arcs[N][N], int n, double ans[N][N])
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	double temp[N][N];
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				for (t = 0; t < n - 1; t++)
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
			}

			ans[j][i] = getA(temp, n - 1);
			if ((i + j) % 2 == 1)
				ans[j][i] = -ans[j][i];
		}
	}
}

int GetMatrixInverse(double src[N][N], int n, double des[N][N])
{
	double flag = getA(src, n);
	double t[N][N];
	if (flag == 0)
		return -1;
	else
	{
		getAStart(src, n, t);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
				des[i][j] = t[i][j] / flag;
		}
	}

	return 0;
}

void vMatrixMulti(double a[4][4], double b[4][1], double c[4][1])
{
	int i, j, k;
	double temp[4] = { 0 };

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			for (k = 0; k < 1; k++)
				temp[i] += a[i][j] * b[j][k];
		}

		for (k = 0; k < 1; k++)
			c[i][k] = temp[i];
	}
}


// 利用规划的起点和目标点的SL坐标、theta角度，建立4个方程，然后解方程，求解L=f(S)三次多项式的四个参数
int vParasSolver(float fStartS, float fEndS, float fSideDisStart, float fSideDisEnd, float fThetaStart, float fThetaEnd, float *pfC, int *iNum)
{
	int i = 0;
	int j = 0;
	double fc[4][1] = { 0.0 };
	double fInverMatrix[4][4] = { 0.0 };
	double fMatrix_a[4][4] = { { 1.0, fStartS, fStartS * fStartS, fStartS * fStartS *fStartS },
	{ 1.0, fEndS, fEndS * fEndS, fEndS * fEndS *fEndS },
	{ 0.0, 1.0, 2 * fStartS, 3 * fStartS * fStartS },
	{ 0.0, 1.0, 2 * fEndS, 3 * fEndS * fEndS } };
	double fMatrix_b[4][1] = { fSideDisStart, fSideDisEnd, tan(fThetaStart), tan(fThetaEnd) };
	GetMatrixInverse(fMatrix_a, 4, fInverMatrix);
	vMatrixMulti(fInverMatrix, fMatrix_b, fc);
	for (i = 0; i < 4; i++)
	{
		*pfC = (float)fc[i][0];
		pfC++;
	}
	*iNum = 4;

	return 0;
}

// 高斯消元法计算得到n次多项式的系数
void gauss_solve(int n, double A[], double x[], double b[])
{
	int i, j, k, r;
	double max;
	for (k = 0; k<n - 1; k++)
	{
		max = fabs(A[k*n + k]);				// find maxmum 
		r = k;
		for (i = k + 1; i<n - 1; i++)
		{
			if (max<fabs(A[i*n + i]))
			{
				max = fabs(A[i*n + i]);
				r = i;
			}
		}

		if (r != k)
		{
			for (i = 0; i<n; i++)			// change array:A[k]&A[r]
			{
				max = A[k*n + i];
				A[k*n + i] = A[r*n + i];
				A[r*n + i] = max;
			}

			max = b[k];						// change array:b[k]&b[r]
			b[k] = b[r];
			b[r] = max;
		}

		for (i = k + 1; i<n; i++)
		{
			for (j = k + 1; j<n; j++)
				A[i*n + j] -= A[i*n + k] * A[k*n + j] / A[k*n + k];
			b[i] -= A[i*n + k] * b[k] / A[k*n + k];
		}
	}

	for (i = n - 1; i >= 0; x[i] /= A[i*n + i], i--)
	{
		for (j = i + 1, x[i] = b[i]; j<n; j++)
			x[i] -= A[i*n + j] * x[j];
	}
}

// n次曲线拟合函数
void polyfit(int n, double x[], double y[], int poly_n, double p[])
{
	int i, j;
	double *tempx, *tempy, *sumxx, *sumxy, *ata;

	tempx = (double *)calloc(n, sizeof(double));
	sumxx = (double *)calloc((poly_n * 2 + 1), sizeof(double));
	tempy = (double *)calloc(n, sizeof(double));
	sumxy = (double *)calloc((poly_n + 1), sizeof(double));
	ata = (double *)calloc((poly_n + 1)*(poly_n + 1), sizeof(double));

	for (i = 0; i<n; i++)
	{
		tempx[i] = 1;
		tempy[i] = y[i];
	}

	for (i = 0; i<2 * poly_n + 1; i++)
	{
		for (sumxx[i] = 0, j = 0; j<n; j++)
		{
			sumxx[i] += tempx[j];
			tempx[j] *= x[j];
		}
	}

	for (i = 0; i<poly_n + 1; i++)
	{
		for (sumxy[i] = 0, j = 0; j<n; j++)
		{
			sumxy[i] += tempy[j];
			tempy[j] *= x[j];
		}
	}

	for (i = 0; i<poly_n + 1; i++)
	{
		for (j = 0; j<poly_n + 1; j++)
			ata[i*(poly_n + 1) + j] = sumxx[i + j];
	}

	gauss_solve(poly_n + 1, ata, p, sumxy);
	free(tempx);
	free(sumxx);
	free(tempy);
	free(sumxy);
	free(ata);
}

// 使用三次多项式L=f(S)，计算pStrSLReferLines中索引号位于 [iId, iOutNum+iId) 区间范围内的参考点的L坐标值
void vCal_l(float *pfC, int iNum, StrSLPoints *pStrSLReferLines, int iId, float *pfL, int iOutNum)
{
	int i = 0;
	float fC[4] = { 0.0 };
	float fS = 0.0;

	for (i = 0; i < iNum; i++)
		fC[i] = *pfC++;

	for (i = 0; i < iOutNum; i++)
	{
		fS = pStrSLReferLines->pSLPoint[i + iId].fS;
		*pfL = fC[0] + fC[1] * fS + fC[2] * fS * fS + fC[3] * fS * fS * fS;
		pfL++;
	}
}

// 将pStrSLReferLines中点的XY坐标和S坐标值赋给RoutePoints中的轨迹点
int iCurtoCar(StrSLPoints *pStrSLReferLines, float *pfL, int iId, int iNum, StrSLPoints *RoutePoints)
{
	int i = 0;
	int j = 1;
	int iRet = 0;
	RoutePoints->iNum = iNum;	

	for (i = 0; i < RoutePoints->iNum; i++)
	{
		RoutePoints->pSLPoint[i].fS = pStrSLReferLines->pSLPoint[i + iId].fS;
		RoutePoints->pSLPoint[i].fX = (pStrSLReferLines->pSLPoint[i + iId].fX - *pfL * sin(pStrSLReferLines->pSLPoint[i + iId].fTheta));
		RoutePoints->pSLPoint[i].fY = (pStrSLReferLines->pSLPoint[i + iId].fY + *pfL * cos(pStrSLReferLines->pSLPoint[i + iId].fTheta));
		RoutePoints->pSLPoint[i].fZ = 0.0;
		pfL++;
	}
	if (debug)
		printf("最后一个规划点的坐标为:X=%f,Y=%f\n", RoutePoints->pSLPoint[iNum - 1].fX, RoutePoints->pSLPoint[iNum - 1].fY);

	return 0;
}

StrPoint InsertPoint(StrPoint strPoint1, StrPoint strPoint2)
{
	StrPoint ReturnPoint;

	ReturnPoint.fX = (strPoint1.fX + strPoint2.fX) / 2;
	ReturnPoint.fY = (strPoint1.fY + strPoint2.fY) / 2;
	ReturnPoint.fZ = (strPoint1.fZ + strPoint2.fZ) / 2;

	return ReturnPoint;
}

float fAngle(float fx, float fy)
{
	float fAngle = 0.0;
	float deltS = sqrt(fx * fx + fy * fy);

	if (fx >= 0.0)
		fAngle = acosf(fx / deltS);
	else
		fAngle = 2 * PI - acosf(fx / deltS);

	return fAngle;
}

// 将参考点的坐标从车体坐标系转换到SL坐标系
int iReferLineXYToSL(Global *pStrXYReferLine, StrLocationFusion *pStrCarStatus, StrSLPoints *pStrSLReferLine)
{
	int i = 0;
	int j = 0;
	float fDeletS = 0.0;
	StrPoint strPoint, strPoint1, strPoint2;
	double x1 = 0.0;
	double x2 = 0.0;
	double x3 = 0.0;
	double y1 = 0.0;
	double y2 = 0.0;
	double y3 = 0.0;

	if (NULL == pStrXYReferLine || NULL == pStrSLReferLine)
	{
		if (debug)
			printf("Param Error\n");
		return -1;
	}

	SLPoint VehiclePosition;
	VehiclePosition.fX = 0.0;
	VehiclePosition.fY = 0.0;

	float fDis = sqrt((VehiclePosition.fX - pStrXYReferLine->POINT[0].X) * (VehiclePosition.fX - pStrXYReferLine->POINT[0].X)
		+ (VehiclePosition.fY - pStrXYReferLine->POINT[0].Y)*(VehiclePosition.fY - pStrXYReferLine->POINT[0].Y));

	pStrSLReferLine->iNum = pStrXYReferLine->NUM_POINT;
	// The first Point
	pStrSLReferLine->pSLPoint[0].fS = 0.0;
	pStrSLReferLine->pSLPoint[0].fL = 0.0;
	pStrSLReferLine->pSLPoint[0].iId = 0;
	pStrSLReferLine->pSLPoint[0].fX = pStrXYReferLine->POINT[0].X;
	pStrSLReferLine->pSLPoint[0].fY = pStrXYReferLine->POINT[0].Y;

	for (i = 1; i < pStrSLReferLine->iNum; i++)
	{
		fDeletS = sqrt((pStrXYReferLine->POINT[i].X - pStrXYReferLine->POINT[i - 1].X)*(pStrXYReferLine->POINT[i].X - pStrXYReferLine->POINT[i - 1].X)
			+ (pStrXYReferLine->POINT[i].Y - pStrXYReferLine->POINT[i - 1].Y)*(pStrXYReferLine->POINT[i].Y - pStrXYReferLine->POINT[i - 1].Y));
		pStrSLReferLine->pSLPoint[i].fS = pStrSLReferLine->pSLPoint[i - 1].fS + fDeletS;
		pStrSLReferLine->pSLPoint[i].fL = 0.0;
		pStrSLReferLine->pSLPoint[i].iId = i;
		pStrSLReferLine->pSLPoint[i].fX = pStrXYReferLine->POINT[i].X;
		pStrSLReferLine->pSLPoint[i].fY = pStrXYReferLine->POINT[i].Y;
	}

	for (i = 1; i < pStrSLReferLine->iNum - 1; i++)
	{
		x1 = pStrSLReferLine->pSLPoint[i - 1].fX;
		x2 = pStrSLReferLine->pSLPoint[i].fX;
		x3 = pStrSLReferLine->pSLPoint[i + 1].fX;
		y1 = pStrSLReferLine->pSLPoint[i - 1].fY;
		y2 = pStrSLReferLine->pSLPoint[i].fY;
		y3 = pStrSLReferLine->pSLPoint[i + 1].fY;
		pStrSLReferLine->pSLPoint[i].fTheta = fThetaFit(x1, x2, x3, y1, y2, y3);	// 车辆方向盘的转向角度，取值范围为：左(正)、右(负)各60度
	}
	// 第一个参考点和最后一个参考点的转向角取值都为0
	pStrSLReferLine->pSLPoint[0].fTheta = 0.0;
	pStrSLReferLine->pSLPoint[pStrSLReferLine->iNum - 1].fTheta = 0.0;

	return 0;
}

// Return the ID of the point in reference line that nearest to the given point in SL coordinate
// 求最近邻点
int iFindNearestPoint(SLPoint *pTargetPoint, StrSLPoints *pStrSLReferLines)
{
	int i = 0;
	float fMinDis = 0.0;
	float fDis = 0.0;
	StrPoint ReturnPoint;
	int iReturnId = 0;;

	if (NULL == pTargetPoint)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	fMinDis = sqrt((pTargetPoint->fX - pStrSLReferLines->pSLPoint[0].fX)*(pTargetPoint->fX - pStrSLReferLines->pSLPoint[0].fX)
		+ (pTargetPoint->fY - pStrSLReferLines->pSLPoint[0].fY)*(pTargetPoint->fY - pStrSLReferLines->pSLPoint[0].fY));
	ReturnPoint.fX = pStrSLReferLines->pSLPoint[0].fX;
	ReturnPoint.fY = pStrSLReferLines->pSLPoint[0].fY;
	iReturnId = pStrSLReferLines->pSLPoint[0].iId;

	for (i = 1; i < pStrSLReferLines->iNum; i++)
	{
		fDis = sqrt((pTargetPoint->fX - pStrSLReferLines->pSLPoint[i].fX)*(pTargetPoint->fX - pStrSLReferLines->pSLPoint[i].fX)
			+ (pTargetPoint->fY - pStrSLReferLines->pSLPoint[i].fY)*(pTargetPoint->fY - pStrSLReferLines->pSLPoint[i].fY));
		if (fMinDis > fDis)
		{
			fMinDis = fDis;
			ReturnPoint.fX = pStrSLReferLines->pSLPoint[i].fX;
			ReturnPoint.fY = pStrSLReferLines->pSLPoint[i].fY;
			iReturnId = pStrSLReferLines->pSLPoint[i].iId;
		}
	}

	return iReturnId;
}

// 根据pTargetPoint的S坐标值，从pStrSLReferLines中查找离规划目标点最近的参考点，函数返回值的大小在[0, 50)区间范围内
int iFindNearestS(SLPoint *pTargetPoint, StrSLPoints *pStrSLReferLines)
{
	int i = 0;
	float fMinDis = 0.0;
	float fDis = 0.0;
	StrPoint ReturnPoint;
	int iReturnId = 0;;

	if (NULL == pTargetPoint)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	fMinDis = fabs(pTargetPoint->fS - pStrSLReferLines->pSLPoint[0].fS);
	ReturnPoint.fX = pStrSLReferLines->pSLPoint[0].fX;
	ReturnPoint.fY = pStrSLReferLines->pSLPoint[0].fY;
	iReturnId = pStrSLReferLines->pSLPoint[0].iId;

	for (i = 1; i < pStrSLReferLines->iNum; i++)
	{
		fDis = fabs(pTargetPoint->fS - pStrSLReferLines->pSLPoint[i].fS);
		if (fMinDis > fDis)
		{
			fMinDis = fDis;
			ReturnPoint.fX = pStrSLReferLines->pSLPoint[i].fX;
			ReturnPoint.fY = pStrSLReferLines->pSLPoint[i].fY;
			iReturnId = pStrSLReferLines->pSLPoint[i].iId;
		}
	}

	return iReturnId;
}

// 跟车模式：保持前进状态，规划点的S坐标设为决策目标点的S坐标，L坐标设为0，速度为最大速度的一半
int iFollowModeTargetFilter(SLPoint *pVehicleSLPoint, SLPoint *pDecisionTarget, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pTargetSLPoints)
{
	int i = 0;
	float fDis = 0.0;
	float fMinDis = 0.0;
	int iId = 0;
	float fS = 0.0;
	float fMinS = 0.0;

	if (NULL == pVehicleSLPoint || NULL == pDecisionTarget || NULL == pStrSLReferLines || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (NULL == pTargetSLPoints)
	{
		if (debug)
			printf("Output Param Error\n");
		return -1;
	}

	pTargetSLPoints->iNum = 1;
	pTargetSLPoints->pSLPoint[0].fS = pDecisionTarget->fS;
	pTargetSLPoints->pSLPoint[0].fL = 0.0;
	pTargetSLPoints->pSLPoint[0].fV = pStrPlanningConfig->fMaxV / 2.0;
	pTargetSLPoints->pSLPoint[0].fX = pDecisionTarget->fX;
	pTargetSLPoints->pSLPoint[0].fY = pDecisionTarget->fY;
	pTargetSLPoints->pSLPoint[0].iId = pDecisionTarget->iId;

	return 0;
}

// 前进模式：使用地图上的参考点作为规划目标点
int iForwardModeTargetFilter(SLPoint *pVehicleSLPoint, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pTargetSLPoints)
{
	int i = 0;
	float fDis = 0.0;
	float fMinDis = 0.0;
	int iId = 0;
	float fS = 0.0;
	float fMinS = 0.0;
	float fMaxS = 0.0;
	float fPlanningDis = pStrPlanningConfig->fMaxPlanningDistance;

	if (NULL == pVehicleSLPoint || NULL == pStrSLReferLines || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (NULL == pTargetSLPoints)
	{
		if (debug)
			printf("Output Param Error\n");
		return -1;
	}

	// 目标个数赋值
	pTargetSLPoints->iNum = 1;
	fMinS = pStrSLReferLines->pSLPoint[0].fS;
	fMaxS = pStrSLReferLines->pSLPoint[pStrSLReferLines->iNum - 1].fS;
	if (fMinS + fPlanningDis > fMaxS)
		pTargetSLPoints->pSLPoint[0].fS = fMaxS;
	else
		pTargetSLPoints->pSLPoint[0].fS = fMinS + fPlanningDis;

	iId = iFindNearestS(&pTargetSLPoints->pSLPoint[0], pStrSLReferLines);
	pTargetSLPoints->pSLPoint[0].fL = 0.0;
	pTargetSLPoints->pSLPoint[0].fTheta = pStrSLReferLines->pSLPoint[iId].fTheta;
	pTargetSLPoints->pSLPoint[0].fV = pStrSLReferLines->pSLPoint[iId].fV;
	pTargetSLPoints->pSLPoint[0].fX = pStrSLReferLines->pSLPoint[iId].fX;
	pTargetSLPoints->pSLPoint[0].fY = pStrSLReferLines->pSLPoint[iId].fY;
	pTargetSLPoints->pSLPoint[0].iId = iId;

	return 0;
}

float fCalRoadWidth(StrTargetFusion *pTargetsFusion, StrPlanningConfig *pStrPlanningConfig)
{
	float fRoadWidth = 0.0;
	int i = 0;
	int j = 0;

	if (0 == pTargetsFusion->strRoadBoard.iRoadPointNum)
	{
		fRoadWidth = pStrPlanningConfig->fLeftHalfRoadWidth + pStrPlanningConfig->fRightHalfRoadWidth;
		return fRoadWidth;
	}

	float fMinRoadwidth = sqrt((pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fX - pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fX)*(pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fX - pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fX)
		+ (pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fY - pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fY) *(pTargetsFusion->strRoadBoard.strLeftRoadBoard[0].fY - pTargetsFusion->strRoadBoard.strRightRoadBoard[0].fY));
	for (i = 0; i < pTargetsFusion->strRoadBoard.iRoadPointNum; i++)
	{
		for (j = 1; j < pTargetsFusion->strRoadBoard.iRoadPointNum; j++)
		{
			fRoadWidth = sqrt((pTargetsFusion->strRoadBoard.strLeftRoadBoard[i].fX - pTargetsFusion->strRoadBoard.strRightRoadBoard[j].fX) *(pTargetsFusion->strRoadBoard.strLeftRoadBoard[i].fX - pTargetsFusion->strRoadBoard.strRightRoadBoard[j].fX)
				+ (pTargetsFusion->strRoadBoard.strLeftRoadBoard[i].fY - pTargetsFusion->strRoadBoard.strRightRoadBoard[j].fY) * (pTargetsFusion->strRoadBoard.strLeftRoadBoard[i].fY - pTargetsFusion->strRoadBoard.strRightRoadBoard[j].fY));
			if (fMinRoadwidth > fRoadWidth)
				fMinRoadwidth = fRoadWidth;
		}
	}

	return fMinRoadwidth;
}

float fFindL(SLPoint *pSLPoint, StrSLPoints *pStrSLReferLines)
{
	int iId = 0;
	float fL = 0.0;
	iId = iFindNearestPoint(pSLPoint, pStrSLReferLines);
	fL = sqrt((pSLPoint->fX - pStrSLReferLines->pSLPoint[iId].fX)* (pSLPoint->fX - pStrSLReferLines->pSLPoint[iId].fX)
		+ (pSLPoint->fY - pStrSLReferLines->pSLPoint[iId].fY) * (pSLPoint->fY - pStrSLReferLines->pSLPoint[iId].fY));
	// 构造向量求解侧向偏差的方向
	float fX = pSLPoint->fX;
	float fY = pSLPoint->fY;
	float fX0 = 0.0;
	float fY0 = 0.0;
	float fX1 = 0.0;
	float fY1 = 0.0;
	float fX2 = 0.0;
	float fY2 = 0.0;

	if (0 == iId)
	{
		fX0 = pStrSLReferLines->pSLPoint[0].fX;
		fY0 = pStrSLReferLines->pSLPoint[0].fY;
		fX1 = pStrSLReferLines->pSLPoint[0].fX;
		fY1 = pStrSLReferLines->pSLPoint[0].fY;
	}
	else
	{
		fX0 = pStrSLReferLines->pSLPoint[iId].fX;
		fY0 = pStrSLReferLines->pSLPoint[iId].fY;
		fX1 = pStrSLReferLines->pSLPoint[iId - 1].fX;
		fY1 = pStrSLReferLines->pSLPoint[iId - 1].fY;
	}

	if (pStrSLReferLines->iNum - 1 == iId)
	{
		fX2 = pStrSLReferLines->pSLPoint[iId].fX;
		fY2 = pStrSLReferLines->pSLPoint[iId].fY;
	}
	else
	{
		fX2 = pStrSLReferLines->pSLPoint[iId + 1].fX;
		fY2 = pStrSLReferLines->pSLPoint[iId + 1].fY;
	}

	float fD = sqrt((fX2 - fX0)*(fX2 - fX0) + (fY2 - fY0)* (fY2 - fY0));
	if (fD == 0.0)
		fD = 0.1;
	float fCos = (fX2 - fX0) / fD;
	float fSin = (fY2 - fY0) / fD;
	float fx = (fX - fX0) * fCos + (fY - fY0) * fSin;
	float fy = (fY - fY0) * fCos - (fX - fX0) * fSin;
	fL = fy;

	return fL;
}

// 绕行模式
int iPassModeTargetFilter(SLPoint *pVehicleSLPoint, SLPoint *pDecisionTarget, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pTargetSLPoints)
{
	int i = 0;
	int iId = 0;

	if (NULL == pDecisionTarget || NULL == pStrSLReferLines || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (NULL == pTargetSLPoints)
	{
		if (debug)
			printf("Output Param Error\n");
		return -1;
	}

	pTargetSLPoints->iNum = 1;
	float fVehicleWidth = pStrPlanningConfig->fVehicleWidth;
	float fL = pDecisionTarget->fL;

	if (Decision == LeftPass)
	{
		//pTargetSLPoints->pSLPoint[0].fS = pDecisionTarget->fS + 2.0 * exp(-1.5 * pDecisionTarget->fS);
		pTargetSLPoints->pSLPoint[0].fS = pDecisionTarget->fS + 0.5;
		//float PlanningL = (pDecisionTarget->fL + 0.5 * pDecisionTarget->fWidth + pStrPlanningConfig->fLeftHalfRoadWidth) * 0.5;
		//float BoundaryL = pStrPlanningConfig->fLeftHalfRoadWidth - 0.5 * pStrPlanningConfig->fVechicleWidth;
		float PlanningL = pDecisionTarget->fL + 0.5 * pDecisionTarget->fWidth + SAFETY_GAP_DISTANCE + 0.5 * pStrPlanningConfig->fVehicleWidth;
		float BoundaryL = pDecisionTarget->fL + 0.5 * pDecisionTarget->fWidth + 1.5 * SAFETY_GAP_DISTANCE + 0.5 * pStrPlanningConfig->fVehicleWidth;
		if (pDecisionTarget->fS >= 6.0)
			pTargetSLPoints->pSLPoint[0].fL = BoundaryL;
			//pTargetSLPoints->pSLPoint[0].fL = PlanningL;
		else if (pDecisionTarget->fS <= 0)
		{
			pTargetSLPoints->pSLPoint[0].fL = 0;
			if (debug)
				printf("错误：决策目标的S坐标小于或等于0 \n");
		}
		else
			pTargetSLPoints->pSLPoint[0].fL = PlanningL * (1 - pDecisionTarget->fS / 6) + BoundaryL * pDecisionTarget->fS / 6;
			//pTargetSLPoints->pSLPoint[0].fL = PlanningL;
	}
	else if (Decision == RightPass)
	{
		//pTargetSLPoints->pSLPoint[0].fS = pDecisionTarget->fS + 2.0 * exp(-1.5 * pDecisionTarget->fS);
		pTargetSLPoints->pSLPoint[0].fS = pDecisionTarget->fS + 0.5;
		//float PlanningL = (pDecisionTarget->fL - 0.5 * pDecisionTarget->fWidth - pStrPlanningConfig->fRightHalfRoadWidth) * 0.5;
		//float BoundaryL = -pStrPlanningConfig->fRightHalfRoadWidth + 0.5 * pStrPlanningConfig->fVechicleWidth;
		float PlanningL = pDecisionTarget->fL - 0.5 * pDecisionTarget->fWidth - SAFETY_GAP_DISTANCE - 0.5 * pStrPlanningConfig->fVehicleWidth;
		float BoundaryL = pDecisionTarget->fL - 0.5 * pDecisionTarget->fWidth - 1.5 * SAFETY_GAP_DISTANCE - 0.5 * pStrPlanningConfig->fVehicleWidth;
		if (pDecisionTarget->fS >= 6.0)
			pTargetSLPoints->pSLPoint[0].fL = BoundaryL;
			//pTargetSLPoints->pSLPoint[0].fL = PlanningL;
		else if (pDecisionTarget->fS <= 0)
		{
			pTargetSLPoints->pSLPoint[0].fL = 0;
			if (debug)
				printf("错误：决策目标的S坐标小于或等于0 \n");
		}
		else
			pTargetSLPoints->pSLPoint[0].fL = PlanningL * (1 - pDecisionTarget->fS / 6) + BoundaryL * pDecisionTarget->fS / 6;
			//pTargetSLPoints->pSLPoint[0].fL = PlanningL;
	}

	if (pTargetSLPoints->pSLPoint[0].fL <= -pStrPlanningConfig->fRightHalfRoadWidth + 0.5 * pStrPlanningConfig->fVehicleWidth)
		pTargetSLPoints->pSLPoint[0].fL = -pStrPlanningConfig->fRightHalfRoadWidth + 0.5 * pStrPlanningConfig->fVehicleWidth;

	if (pTargetSLPoints->pSLPoint[0].fL >= pStrPlanningConfig->fLeftHalfRoadWidth - 0.5 * pStrPlanningConfig->fVehicleWidth)
		pTargetSLPoints->pSLPoint[0].fL = pStrPlanningConfig->fLeftHalfRoadWidth - 0.5 * pStrPlanningConfig->fVehicleWidth;

	if (debug)
		printf("规划目标点的SL坐标为: S=%f, L=%f \n", pTargetSLPoints->pSLPoint[0].fS, pTargetSLPoints->pSLPoint[0].fL);
	pTargetSLPoints->pSLPoint[0].fTheta = 0.0;
	iId = iFindNearestS(&pTargetSLPoints->pSLPoint[0], pStrSLReferLines);
	pTargetSLPoints->pSLPoint[0].fX = pStrSLReferLines->pSLPoint[iId].fX;
	pTargetSLPoints->pSLPoint[0].fY = pStrSLReferLines->pSLPoint[iId].fY;
	pTargetSLPoints->pSLPoint[0].iId = iId;

	return 0;
}

float fGetObstacleWidth(StrTargetStatus *pTargetStatus, StrSLPoints *pStrSLReferLines)
{
	float fMaxWidth = 0.0;
	float fL[4] = { 0.0 };
	int i = 0;
	SLPoint TargetSL;
	for (i = 0; i < 4; i++)
	{
		TargetSL.fX = pTargetStatus->strTargetBoard.strTargetPoint[i].fX;
		TargetSL.fY = pTargetStatus->strTargetBoard.strTargetPoint[i].fY;
		fL[i] = fFindL(&TargetSL, pStrSLReferLines);
	}

	float fWidth[6] = { 0.0 };

	for (i = 0; i < 3; i++)
		fWidth[i] = fabs(fL[0] - fL[i + 1]);

	for (i = 1; i < 3; i++)
		fWidth[i + 2] = fabs(fL[1] - fL[i + 1]);

	fWidth[5] = fabs(fL[2] - fL[3]);
	fMaxWidth = fWidth[0];

	for (i = 1; i < 5; i++)
	{
		if (fWidth[i] >= fMaxWidth)
			fMaxWidth = fWidth[i];
	}

	return fMaxWidth;
}

// 毫米波决策过程：当且仅当障碍物与本车在参考线的相同侧，且两者之间的侧向距离小于1.2米、纵向距离小于2.5米时，毫米波决策为安全停车
int iRadarTargetFilter(StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, EmDecision *RadarDecision, StrTargetFusion *pRadarTargetsFilterFusion)
{
	if ((NULL == pTargetsFusion) || (NULL == pRadarTargetsFilterFusion))
	{
		if (debug)
			printf("Input Error\n");
		return -1;
	}

	int i = 0;
	int j = 0;
	float RadarTargetfL = 0.0;
	float RadarTargetfS = 0.0;
	int iId = 0;

	if (0 == pTargetsFusion->iTargetNum)
	{
		if (debug)
			printf("所有传感器都没有检测到障碍物 \n");
		pRadarTargetsFilterFusion->iTargetNum = 0;
		return 0;
	}

	// 从pTargetsFusion提取超声波，放入到pRadarTargetsFilterFusion
	pRadarTargetsFilterFusion->strUltrasonicDatas = pTargetsFusion->strUltrasonicDatas;
	SLPoint TargetSL;
	// 这里单独针对毫米波进行分析，只要毫米波决策不是安全停车，就把pTargetsFusion中的障碍物数据传递给pRadarTargetsFilterFusion
	for (i = 0; i < pTargetsFusion->iTargetNum; i++)
	{
		if (pTargetsFusion->strTargetStatus[i].emSensorType != SENSOR_RADAR)	// 如果不是毫米波雷达
		{
			pRadarTargetsFilterFusion->strTargetStatus[j] = pTargetsFusion->strTargetStatus[i];
			j++;
			pRadarTargetsFilterFusion->iTargetNum = j;
		}
		else	// 如果是毫米波雷达，有障碍物目标的坐标信息
		{
			TargetSL.fX = pTargetsFusion->strTargetStatus[i].fX;
			TargetSL.fY = pTargetsFusion->strTargetStatus[i].fY;

			RadarTargetfL = fFindL(&TargetSL, pStrSLReferLines);	// 毫米波雷达检测的障碍物的L坐标
			iId = iFindNearestPoint(&TargetSL, pStrSLReferLines);	// 毫米波雷达检测的障碍物最近的参考点
			RadarTargetfS = pStrSLReferLines->pSLPoint[iId].fS;		// 毫米波雷达检测的障碍物的S坐标,近似为最近参考点的S坐标
			if (RadarTargetfS - VehicleSLPoint.fS <= 1.5)
			{
				if ((RadarTargetfL > 0.0 && VehicleSLPoint.fL > 0.0) || (RadarTargetfL <= 0.0 && VehicleSLPoint.fL <= 0.0))
				{
					if (fabs(RadarTargetfL - VehicleSLPoint.fL) <= 1.2)		// 横向安全距离小于1.2m
					{
						*RadarDecision = Safestop;
						if (debug)
							printf("毫米波检测障碍物与本车在参考线的相同侧，两者之间的纵向距离小于2.5米，侧向距离小于1.2米，毫米波决策为安全停车\n");
						return 0;
					}
					else
					{
						pRadarTargetsFilterFusion->strTargetStatus[j] = pTargetsFusion->strTargetStatus[i];
						j++;
						pRadarTargetsFilterFusion->iTargetNum = j;
					}
				}
				else
				{
					pRadarTargetsFilterFusion->strTargetStatus[j] = pTargetsFusion->strTargetStatus[i];
					j++;
					pRadarTargetsFilterFusion->iTargetNum = j;
				}
			}
		}
	}
	if (debug)
		printf("有效范围内的障碍物数量为:%d\n", pRadarTargetsFilterFusion->iTargetNum);

	return 0;
}

// 在iTargetFilter函数中，只是从激光雷达和毫米波雷达的障碍物中选取决策目标，从而确定决策点的位置，而没有从超声波雷达的障碍物中选取（超声波雷达的障碍物数据不包含障碍物的XY坐标，只有障碍物的距离）
EmDecision iTargetFilter(StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, StrPlanningConfig *pStrPlanningConfig, SLPoint *pDecisionPoint)
{
	int i = 0;
	int iId = 0;
	float fMinS1 = pStrPlanningConfig->fMaxPlanningDistance;
	float fMinS2 = pStrPlanningConfig->fMaxPlanningDistance;
	float fS = 0.0;
	int iTargetId1[MAX_TARGET_NUM];
	float fTargetL1[MAX_TARGET_NUM];
	int iTargetId2[MAX_TARGET_NUM];
	float fTargetL2[MAX_TARGET_NUM];
	memset(&TargetSLPoints, 0, sizeof(StrSLPoints));
	TargetSLPoints.iNum = 2;		// 参考线两边各一个最有危险障碍物
	LeftTargetNum = 0;
	RightTargetNum = 0;
	EmDecision Decision;
	SLPoint DecisionPoint;

	if (0 == pTargetsFusion->iTargetNum)
	{
		if (debug)
			printf("在有效范围内，车辆周围没有障碍物 \n");
	}
	else
	{
		for (i = 0; i < pTargetsFusion->iTargetNum; i++)
		{
			if (debug)
				// 每个pTargetsFusion->strTargetStatus[i]障碍物的XYZ坐标是在当前车体坐标系下的，问题是：是在什么时候进行转换的？
				printf("第[%d]个障碍物的坐标: X=%f, Y=%f, 宽度=%f，类型=%s \n", i, pTargetsFusion->strTargetStatus[i].fX, pTargetsFusion->strTargetStatus[i].fY, pTargetsFusion->strTargetStatus[i].fWidth, Sensor_Type_String[pTargetsFusion->strTargetStatus[i].emSensorType].c_str());
		}
	}

	EmSensorType SensorType[2] = { SENSOR_UNKNOW };
	if (0 == pTargetsFusion->iTargetNum)
	{
		if (debug)
			printf("Decision 1\n");
		Decision = Forward;
	}
	else
	{
		SLPoint FirstPoint;
		SLPoint Point;
		float fWidth0 = 0.5;		// 障碍物1的宽度
		float fWidth1 = 0.5;		// 障碍物2的宽度
		float fWidth = 0.0;			// 最近障碍物的宽度

		// 根据本车fL的正负，确定本车正前方和参考线对侧的障碍物的最短距离
		SLPoint TargetSL;
		for (i = 0; i < pTargetsFusion->iTargetNum; i++)
		{
			TargetSL.fX = pTargetsFusion->strTargetStatus[i].fX;
			TargetSL.fY = pTargetsFusion->strTargetStatus[i].fY;
			// 激光雷达的输出包含包围盒四个点的坐标，但根据这些数据可以推算出目标的宽度
			if (pTargetsFusion->strTargetStatus[i].emSensorType == SENSOR_LIDAR)
			{
				fWidth = fGetObstacleWidth(&pTargetsFusion->strTargetStatus[i], pStrSLReferLines);
				pTargetsFusion->strTargetStatus[i].fWidth = fWidth;
			}
			// 毫米波雷达的输出包含目标的宽度
			else if (pTargetsFusion->strTargetStatus[i].emSensorType == SENSOR_RADAR)
			{
				fWidth = pTargetsFusion->strTargetStatus[i].fWidth;
				if (fWidth <= 0.5)
				{
					fWidth = 0.5;
					pTargetsFusion->strTargetStatus[i].fWidth = 0.5;
				}
			}
			// 对于超声波雷达而言，只有距离大小，没有目标的宽度
			// 计算目标的fL坐标
			float fL = fFindL(&TargetSL, pStrSLReferLines);
			// 过滤掉车道外面的障碍物
			if (((fL - 0.5 * fWidth) < pStrPlanningConfig->fLeftHalfRoadWidth) && ((fL + 0.5 * fWidth) > -pStrPlanningConfig->fRightHalfRoadWidth))
			{
				if (fL > 0)
				{
					iTargetId1[LeftTargetNum] = i;
					fTargetL1[LeftTargetNum] = fL;
					LeftTargetNum++;
					if (debug)
						printf("参考线左侧目标的坐标:X = %f,Y = %f,L = %f\n", TargetSL.fX, TargetSL.fY, fL);
				}
				else
				{
					iTargetId2[RightTargetNum] = i;
					fTargetL2[RightTargetNum] = fL;
					RightTargetNum++;
					if (debug)
						printf("参考线右侧目标的坐标:X = %f,Y = %f,L = %f\n", TargetSL.fX, TargetSL.fY, fL);
				}
			}
		}

		if (LeftTargetNum > 0)
		{
			FirstPoint.fX = pTargetsFusion->strTargetStatus[iTargetId1[0]].fX;
			FirstPoint.fY = pTargetsFusion->strTargetStatus[iTargetId1[0]].fY;
			iId = iFindNearestPoint(&FirstPoint, pStrSLReferLines);
			fMinS1 = pStrSLReferLines->pSLPoint[iId].fS;
			TargetSLPoints.pSLPoint[0].fS = fMinS1;
			TargetSLPoints.pSLPoint[0].fL = fTargetL1[0];
			TargetSLPoints.pSLPoint[0].fTheta = pStrSLReferLines->pSLPoint[iId].fTheta;
			TargetSLPoints.pSLPoint[0].fV = pStrSLReferLines->pSLPoint[iId].fV;
			TargetSLPoints.pSLPoint[0].fX = FirstPoint.fX;
			TargetSLPoints.pSLPoint[0].fY = FirstPoint.fY;
			TargetSLPoints.pSLPoint[0].iId = iId;
			TargetSLPoints.pSLPoint[0].fWidth = pTargetsFusion->strTargetStatus[iTargetId1[0]].fWidth;
			TargetSLPoints.pSLPoint[0].fLength = pTargetsFusion->strTargetStatus[iTargetId1[0]].fLength;
			fWidth0 = pTargetsFusion->strTargetStatus[iTargetId1[0]].fWidth;
			SensorType[0] = pTargetsFusion->strTargetStatus[iTargetId1[0]].emSensorType;

			for (i = 1; i < LeftTargetNum; i++)
			{
				Point.fX = pTargetsFusion->strTargetStatus[iTargetId1[i]].fX;
				Point.fY = pTargetsFusion->strTargetStatus[iTargetId1[i]].fY;
				iId = iFindNearestPoint(&Point, pStrSLReferLines);
				fS = pStrSLReferLines->pSLPoint[iId].fS;
				if (fMinS1 > fS)
				{
					fMinS1 = fS;
					TargetSLPoints.pSLPoint[0].fS = fMinS1;
					TargetSLPoints.pSLPoint[0].fL = fTargetL1[i];
					TargetSLPoints.pSLPoint[0].fTheta = pStrSLReferLines->pSLPoint[iId].fTheta;
					TargetSLPoints.pSLPoint[0].fV = pStrSLReferLines->pSLPoint[iId].fV;
					TargetSLPoints.pSLPoint[0].fX = Point.fX;
					TargetSLPoints.pSLPoint[0].fY = Point.fY;
					TargetSLPoints.pSLPoint[0].iId = iId;
					TargetSLPoints.pSLPoint[0].fWidth = pTargetsFusion->strTargetStatus[iTargetId1[i]].fWidth;
					TargetSLPoints.pSLPoint[0].fLength = pTargetsFusion->strTargetStatus[iTargetId1[i]].fLength;
					fWidth0 = pTargetsFusion->strTargetStatus[iTargetId1[i]].fWidth;
					SensorType[0] = pTargetsFusion->strTargetStatus[iTargetId1[i]].emSensorType;
				}
			}
			if (debug)
				printf("参考线左侧最近目标:fS = %f,fL = %f,fX = %f,fY = %f,width = %f,Id = %d\n", TargetSLPoints.pSLPoint[0].fS, TargetSLPoints.pSLPoint[0].fL, TargetSLPoints.pSLPoint[0].fX, TargetSLPoints.pSLPoint[0].fY, fWidth0, TargetSLPoints.pSLPoint[0].iId);
			if (SensorType[0] == SENSOR_LIDAR)
			{
				if (debug)
					printf("参考线左侧最近目标的传感器类型是: 激光雷达 \n");
			}
			else if (SensorType[0] == SENSOR_RADAR)
			{
				if (debug)
					printf("参考线左侧最近目标的传感器类型是: 毫米波雷达 \n");
			}
			else if (SensorType[0] == SENSOR_UNKNOW)
			{
				if (debug)
					printf("参考线左侧最近目标的传感器类型是: 未知的 \n");
			}	
		}

		if (RightTargetNum > 0)
		{
			FirstPoint.fX = pTargetsFusion->strTargetStatus[iTargetId2[0]].fX;
			FirstPoint.fY = pTargetsFusion->strTargetStatus[iTargetId2[0]].fY;
			iId = iFindNearestPoint(&FirstPoint, pStrSLReferLines);
			fMinS2 = pStrSLReferLines->pSLPoint[iId].fS;
			TargetSLPoints.pSLPoint[1].fS = fMinS2;
			TargetSLPoints.pSLPoint[1].fL = fTargetL2[0];
			TargetSLPoints.pSLPoint[1].fTheta = pStrSLReferLines->pSLPoint[iId].fTheta;
			TargetSLPoints.pSLPoint[1].fV = pStrSLReferLines->pSLPoint[iId].fV;
			TargetSLPoints.pSLPoint[1].fX = FirstPoint.fX;
			TargetSLPoints.pSLPoint[1].fY = FirstPoint.fY;
			TargetSLPoints.pSLPoint[1].iId = iId;
			TargetSLPoints.pSLPoint[1].fWidth = pTargetsFusion->strTargetStatus[iTargetId2[0]].fWidth;
			TargetSLPoints.pSLPoint[1].fLength = pTargetsFusion->strTargetStatus[iTargetId2[0]].fLength;
			fWidth1 = pTargetsFusion->strTargetStatus[iTargetId2[0]].fWidth;
			SensorType[1] = pTargetsFusion->strTargetStatus[iTargetId2[0]].emSensorType;

			for (i = 1; i < RightTargetNum; i++)
			{
				Point.fX = pTargetsFusion->strTargetStatus[iTargetId2[i]].fX;
				Point.fY = pTargetsFusion->strTargetStatus[iTargetId2[i]].fY;
				iId = iFindNearestPoint(&Point, pStrSLReferLines);
				fS = pStrSLReferLines->pSLPoint[iId].fS;
				if (fMinS2 > fS)
				{
					fMinS2 = fS;
					TargetSLPoints.pSLPoint[1].fS = fMinS2;
					TargetSLPoints.pSLPoint[1].fL = fTargetL2[i];
					TargetSLPoints.pSLPoint[1].fTheta = pStrSLReferLines->pSLPoint[iId].fTheta;
					TargetSLPoints.pSLPoint[1].fV = pStrSLReferLines->pSLPoint[iId].fV;
					TargetSLPoints.pSLPoint[1].fX = Point.fX;
					TargetSLPoints.pSLPoint[1].fY = Point.fY;
					TargetSLPoints.pSLPoint[1].iId = iId;
					TargetSLPoints.pSLPoint[1].fWidth = pTargetsFusion->strTargetStatus[iTargetId2[i]].fWidth;
					TargetSLPoints.pSLPoint[1].fLength = pTargetsFusion->strTargetStatus[iTargetId2[i]].fLength;
					fWidth1 = pTargetsFusion->strTargetStatus[iTargetId2[i]].fWidth;
					SensorType[1] = pTargetsFusion->strTargetStatus[iTargetId2[i]].emSensorType;
				}
			}
			if (debug)
				printf("参考线右边最近目标:fS = %f,fL = %f,fX = %f,fY = %f,width = %f,Id = %d\n", TargetSLPoints.pSLPoint[1].fS, TargetSLPoints.pSLPoint[1].fL, TargetSLPoints.pSLPoint[1].fX, TargetSLPoints.pSLPoint[1].fY, fWidth1, TargetSLPoints.pSLPoint[1].iId);
			if (SensorType[1] == SENSOR_LIDAR)
			{
				if (debug)
					printf("参考线右侧最近目标的传感器类型是: 激光雷达 \n");
			}
			else if (SensorType[1] == SENSOR_RADAR)
			{
				if (debug)
					printf("参考线右侧最近目标的传感器类型是: 毫米波雷达 \n");
			}
			else if (SensorType[1] == SENSOR_UNKNOW)
			{
				if (debug)
					printf("参考线右侧最近目标的传感器类型是: 未知的 \n");
			}
		}
		if (debug)
			printf("参考线左侧目标的数量为: %d, 参考线右侧目标的数量为: %d\n", LeftTargetNum, RightTargetNum);
		

		SLPoint DecisionTarget;
		if (LeftTargetNum == 0 && RightTargetNum == 0)				// 所有障碍物都在车道外面
		{
			if (debug)
				printf("所有障碍物都在道路边界外面，决策为:前进 \n");
			Decision = Forward;
			return Decision;
		}
		else if (LeftTargetNum > 0 && RightTargetNum == 0)
		{
			if (debug)
				printf("只有左侧有障碍物，选择左边最近障碍物作为决策目标 \n");
			DecisionTarget = TargetSLPoints.pSLPoint[0];
			fWidth = fWidth0;
		}
		else if (RightTargetNum > 0 && LeftTargetNum == 0)
		{
			if (debug)
				printf("只有右侧有障碍物，选择右边最近障碍物作为决策目标 \n");
			DecisionTarget = TargetSLPoints.pSLPoint[1];
			fWidth = fWidth1;
		}
		else
		{
			if (fMinS1 < fMinS2)
			{
				if (debug)
					printf("两侧都有障碍物，选择左边最近障碍物作为决策目标 \n");
				DecisionTarget = TargetSLPoints.pSLPoint[0];
				fWidth = fWidth0;
			}
			else
			{
				if (debug)
					printf("两侧都有障碍物，选择右边最近障碍物作为决策目标 \n");
				DecisionTarget = TargetSLPoints.pSLPoint[1];
				fWidth = fWidth1;
			}
			if (debug)
				printf("左侧最近障碍物的S坐标为:%f,右侧最近障碍物的S坐标为:%f \n", fMinS1, fMinS2);
		}

		if (debug)
			printf("决策目标的SL坐标:S=%f,L=%f \n", DecisionTarget.fS, DecisionTarget.fL);
		if (debug)
			printf("本车的SL坐标:S=%f,L=%f \n", VehicleSLPoint.fS, VehicleSLPoint.fL);

		if (fabs(DecisionTarget.fL) < 0.5 * fWidth)				// 决策目标压在参考线上
			TargetDeviationDis = 0.5 * fWidth - fabs(DecisionTarget.fL);
		else
			TargetDeviationDis = 0.0;

		if (UltrasonicDecision == LeftPass || UltrasonicDecision == RightPass)
		{
			if (DecisionTarget.fS > pStrPlanningConfig->fSafeDis1)
			{
				*pDecisionPoint = UltrasonicDecisionPoint;
				pDecisionPoint->fWidth = 0;
				Decision = UltrasonicDecision;
				if (debug)
					printf("超声波决策为绕行，而且前方障碍物目标的S坐标大于安全距离，所以总体决策为绕行 \n");
				return Decision;
			}
			else
			{
				if (UltrasonicDecision == LeftPass)				// 往超声波障碍物的左侧绕行
				{
					if (DecisionTarget.fL < VehicleSLPoint.fL)	// 前方障碍物在本车的右侧
					{
						*pDecisionPoint = UltrasonicDecisionPoint;
						pDecisionPoint->fWidth = 0;
						Decision = LeftPass;
						if (debug)
							printf("超声波决策为左侧绕行，前方障碍物在本车的右侧，总体决策为往左绕行 \n");
						return Decision;
					}
					else
					{
						Decision = Safestop;
						if (debug)
							printf("超声波决策为左侧绕行，而前方障碍物在本车的左侧，无法绕行，总体决策为安全停车\n");
						return Decision;
					}
				}
				else if (UltrasonicDecision == RightPass)		// 往超声波障碍物的右侧绕行
				{
					if (DecisionTarget.fL > VehicleSLPoint.fL)	// 前方障碍物在本车的左侧
					{
						*pDecisionPoint = UltrasonicDecisionPoint;
						pDecisionPoint->fWidth = 0;
						Decision = RightPass;
						if (debug)
							printf("超声波决策为右侧绕行，前方障碍物在本车的左侧，总体决策为往右绕行 \n");
						return Decision;
					}
					else
					{
						Decision = Safestop;
						if (debug)
							printf("超声波决策为右侧绕行，而前方障碍物在本车的右侧，无法绕行，总体决策为安全停车\n");
						return Decision;
					}
				}
			}
		}

		// 根据最近目标点做决策
		if (DecisionTarget.fS >= pStrPlanningConfig->fMaxPlanningDistance)
		{
			if (debug)
				printf("决策目标的S坐标大于最大规划距离，决策为前进\n");
			Decision = Forward;
		}
		else if (DecisionTarget.fS < pStrPlanningConfig->fMaxPlanningDistance && DecisionTarget.fS > pStrPlanningConfig->fSafeDis2)
		{
			if (debug)
				printf("决策目标的S坐标小于最大规划距离，但是大于最小安全距离，决策为跟车\n");
			Decision = Follow;
			DecisionPoint = DecisionTarget;
		}
		else if (DecisionTarget.fS > pStrPlanningConfig->fSafeDis1 && DecisionTarget.fS <= pStrPlanningConfig->fSafeDis2)		// Pass or Stop
		{
			if (debug)
				printf("决策目标的S坐标大于安全距离，小于最小安全距离\n");

			if (LeftTargetNum > 0 && RightTargetNum > 0)				// 左右两侧都有障碍物
			{
				if (debug)
					printf("参考线两侧都有障碍物，左侧障碍物的S坐标为%f，右侧障碍物的S坐标为%f\n", fMinS1, fMinS2);

				if (fabs(TargetSLPoints.pSLPoint[0].fL) > (0.5 * fWidth0 + 0.5 * pStrPlanningConfig->fVehicleWidth))
				{
					if (debug)
						printf("左侧障碍物与参考线之间在侧方向上有足够的距离\n");
				}
				else
				{
					if (debug)
						printf("左侧障碍物与参考线之间在侧方向的距离太近了\n");
				}

				if (fabs(TargetSLPoints.pSLPoint[1].fL) > (0.5 * fWidth1 + 0.5 * pStrPlanningConfig->fVehicleWidth))
				{
					if (debug)
						printf("右侧障碍物与参考线之间在侧方向上有足够的距离\n");
				}
				else
				{
					if (debug)
						printf("右侧障碍物与参考线之间在侧方向的距离太近了\n");
				}

				if (fMinS1 <= pStrPlanningConfig->fSafeDis2 && fMinS2 <= pStrPlanningConfig->fSafeDis2)
				{
					if (debug)
						printf("左右两侧障碍物都小于最小安全距离\n");
					if ((fabs(TargetSLPoints.pSLPoint[0].fL) - (0.5 * fWidth0 + 0.5 * pStrPlanningConfig->fVehicleWidth) > SAFETY_GAP_DISTANCE) && (fabs(TargetSLPoints.pSLPoint[1].fL) - (0.5 * fWidth1 + 0.5 * pStrPlanningConfig->fVehicleWidth) > SAFETY_GAP_DISTANCE))
					{
						if (debug)
							printf("左右两侧障碍物都离参考线足够远，决策为前进\n");
						Decision = Forward;
					}
					else
					{
						if (debug)
							printf("左右两侧其中有一侧障碍物离参考线较近，决策为安全停车\n");
						Decision = Safestop;
					}
				}
				else if (fMinS1 <= pStrPlanningConfig->fSafeDis2 && fMinS2 > pStrPlanningConfig->fSafeDis2)
				{
					if (debug)
						printf("左侧障碍物小于最小安全距离，而右侧障碍物大于最小安全距离\n");
					DecisionPoint = TargetSLPoints.pSLPoint[0];
					if (fMinS2 - fMinS1 <= 5.0)
					{
						if (debug)
							printf("左右两侧障碍物之间的纵向距离小于5米\n");
						if ((fabs(TargetSLPoints.pSLPoint[0].fL) - (0.5 * fWidth0 + 0.5 * pStrPlanningConfig->fVehicleWidth) > 0.2) && (fabs(TargetSLPoints.pSLPoint[1].fL) - (0.5 * fWidth1 + 0.5 * pStrPlanningConfig->fVehicleWidth) > 0.2))
						{
							if (debug)
								printf("两侧的障碍物与参考线之间都有足够的距离，决策为前进\n");
							Decision = Forward;
						}
						else
						{
							if (debug)
								printf("左右两侧其中有一侧障碍物离参考线太近了，决策为安全停车\n");
							Decision = Safestop;
						}
					}
					else
					{
						if (debug)
							printf("左右两侧障碍物之间的纵向距离大于5米\n");
						if ((fabs(TargetSLPoints.pSLPoint[0].fL) - (0.5 * fWidth0 + 0.5 * pStrPlanningConfig->fVehicleWidth) > 1.5 * SAFETY_GAP_DISTANCE))
						{
							Decision = Forward;
							if (debug)
								printf("障碍物离参考线足够远，直行\n");
						}
						else
						{
							if (DecisionPoint.fL > 0)
							{
								if ((DecisionPoint.fL - 0.5 * fWidth  + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
								{
									Decision = RightPass;
									if (debug)
										printf("车从障碍物的右侧绕行\n");
								}
								else if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
								{
									Decision = LeftPass;
									if (debug)
										printf("车从障碍物的左侧绕行\n");
								}
								else
								{
									Decision = Safestop;
									if (debug)
										printf("障碍物两侧的距离都不够，安全停车\n");
								}
							}
							else
							{
								if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
								{
									Decision = LeftPass;
									if (debug)
										printf("车从障碍物的左侧绕行\n");
								}
								else if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
								{
									Decision = RightPass;
									if (debug)
										printf("车从障碍物的右侧绕行\n");
								}
								else
								{
									Decision = Safestop;
									if (debug)
										printf("障碍物两侧的距离都不够，安全停车\n");
								}
							}
						}
					}
				}
				else if (fMinS1 > pStrPlanningConfig->fSafeDis2 && fMinS2 <= pStrPlanningConfig->fSafeDis2)
				{
					if (debug)
						printf("左侧障碍物大于最小安全距离，而右侧障碍物小于最小安全距离\n");
					DecisionPoint = TargetSLPoints.pSLPoint[1];
					if (fMinS1 - fMinS2 <= 5.0)
					{
						if (debug)
							printf("左右两侧障碍物之间的纵向距离小于5米，");
						if ((fabs(TargetSLPoints.pSLPoint[0].fL) - (0.5 * fWidth0 + 0.5 * pStrPlanningConfig->fVehicleWidth) > SAFETY_GAP_DISTANCE) && (fabs(TargetSLPoints.pSLPoint[1].fL) - (0.5 * fWidth1 + 0.5 * pStrPlanningConfig->fVehicleWidth) > SAFETY_GAP_DISTANCE))
						{
							if (debug)
								printf("两侧的障碍物与参考线之间都有足够的距离，决策为前进\n");
							Decision = Forward;
						}
						else
						{
							if (debug)
								printf("左右两侧其中有一侧障碍物离参考线太近了，决策为安全停车\n");
							Decision = Safestop;
						}
					}
					else
					{
						if (debug)
							printf("左右两侧障碍物之间的纵向距离大于5米，");
						if (DecisionPoint.fL > 0)
						{
							if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
							{
								Decision = RightPass;
								if (debug)
									printf("车从障碍物的右侧绕行\n");
							}
							else if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
							{
								Decision = LeftPass;
								if (debug)
									printf("车从障碍物的左侧绕行\n");
							}
							else
							{
								Decision = Safestop;
								if (debug)
									printf("障碍物两侧的距离都不够，安全停车\n");
							}
						}
						else
						{
							if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
							{
								Decision = LeftPass;
								if (debug)
									printf("车从障碍物的左侧绕行\n");
							}
							else if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
							{
								Decision = RightPass;
								if (debug)
									printf("车从障碍物的右侧绕行\n");
							}
							else
							{
								Decision = Safestop;
								if (debug)
									printf("障碍物两侧的距离都不够，安全停车\n");
							}
						}
					}
				}
			}
			else // 只有一侧有障碍物
			{
				if (LeftTargetNum > 0)
				{
					if (debug)
						printf("决策目标在左侧, ");
				}
				else if (RightTargetNum > 0)
				{
					if (debug)
						printf("决策目标在右侧, ");
				}

				if ((fabs(DecisionTarget.fL) - (0.5 * fWidth + 0.5 * pStrPlanningConfig->fVehicleWidth) > SAFETY_GAP_DISTANCE))
				{
					if (debug)
						printf("决策目标与参考线之间有足够的距离，决策为前进 \n");
					Decision = Forward;
				}
				else
				{
					if (debug)
						printf("决策目标离参考线较近, ");
					DecisionPoint = DecisionTarget;
					if (DecisionPoint.fL > 0)
					{
						if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
						{
							Decision = RightPass;
							if (debug)
								printf("车从障碍物的右侧绕行\n");
						}
						else if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
						{
							Decision = LeftPass;
							if (debug)
								printf("车从障碍物的左侧绕行\n");
						}
						else
						{
							Decision = Safestop;
							if (debug)
								printf("障碍物两侧的距离都不够，安全停车\n");
						}
					}
					else
					{
						if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
						{
							Decision = LeftPass;
							if (debug)
								printf("车从障碍物的左侧绕行\n");
						}
						else if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
						{
							Decision = RightPass;
							if (debug)
								printf("车从障碍物的右侧绕行\n");
						}
						else
						{
							Decision = Safestop;
							if (debug)
								printf("障碍物两侧的距离都不够，安全停车\n");
						}
					}
				}
			}
		}
		else if (DecisionTarget.fS <= pStrPlanningConfig->fSafeDis1)		// 安全距离为2.5米
		{
			DecisionPoint = DecisionTarget;
			if (DecisionTarget.fS <= 0.0001)
			{
				if (debug)
					printf("车辆通过了决策目标，决策为前进\n");
				Decision = Forward;
			}

			if (debug)
				printf("决策目标的S坐标值小于安全距离2.5米, ");
			// 决策目标在车的左侧，只考虑往障碍物右侧绕行的可能性，否则停车
			if (DecisionPoint.fL > VehicleSLPoint.fL)
			{
				if ((DecisionPoint.fL - 0.5 * fWidth + pStrPlanningConfig->fRightHalfRoadWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
				{
					Decision = RightPass;
					if (debug)
						printf("车从障碍物的右侧绕行\n");
				}
				else
				{
					Decision = Safestop;
					if (debug)
						printf("障碍物两侧的距离都不够，安全停车\n");
				}
			}
			// 决策目标在车的右侧，只考虑往障碍物左侧绕行的可能性，否则停车
			else
			{
				if ((pStrPlanningConfig->fLeftHalfRoadWidth - DecisionPoint.fL - 0.5 * fWidth) > (pStrPlanningConfig->fVehicleWidth + 1.5 * SAFETY_GAP_DISTANCE))
				{
					Decision = LeftPass;
					if (debug)
						printf("车从障碍物的左侧绕行\n");
				}
				else
				{
					Decision = Safestop;
					if (debug)
						printf("障碍物两侧的距离都不够，安全停车\n");
				}
			}
		}
		pDecisionPoint->fWidth = fWidth;
	}

	pDecisionPoint->fS = DecisionPoint.fS;
	pDecisionPoint->fL = DecisionPoint.fL;
	pDecisionPoint->fX = DecisionPoint.fX;
	pDecisionPoint->fY = DecisionPoint.fY;
	pDecisionPoint->iId = DecisionPoint.iId;

	return Decision;
}

int iTargetSelect(SLPoint *pVehicleSLPoint, StrTargetFusion *pTargetsFusion, StrSLPoints *pStrSLReferLines, EmDecision *pEmDecision, SLPoint *pDecisionTarget, StrPlanningConfig *pStrPlanningConfig, StrSLPoints *pTargetSLPoints)
{
	int i = 0;
	EmDecision Decision = *pEmDecision;

	if ((NULL == pVehicleSLPoint) || (NULL == pTargetsFusion) || (NULL == pStrSLReferLines) || (NULL == pStrPlanningConfig))
	{
		if (debug)
			printf("Input param Error\n");
		return -1;
	}

	if (Forward == Decision)
		iForwardModeTargetFilter(pVehicleSLPoint, pStrSLReferLines, pStrPlanningConfig, pTargetSLPoints);
	else if (Follow == Decision)
		iFollowModeTargetFilter(pVehicleSLPoint, pDecisionTarget, pStrSLReferLines, pStrPlanningConfig, pTargetSLPoints);
	else if (LeftPass == Decision || RightPass == Decision)
		iPassModeTargetFilter(pVehicleSLPoint, pDecisionTarget, pStrSLReferLines, pStrPlanningConfig, pTargetSLPoints);
	else if (Safestop == Decision)
	{
		if (debug)
			printf("决策为安全停车\n");
		return -1;
	}
	else
	{
		if (debug)
			printf("决策为退出\n");
		return -1;
	}
	if (debug)
		printf("规划的目标点坐标为:S=%f,L=%f,X=%f,Y=%f,Id=%d \n", pTargetSLPoints->pSLPoint[0].fS, pTargetSLPoints->pSLPoint[0].fL, pTargetSLPoints->pSLPoint[0].fX, pTargetSLPoints->pSLPoint[0].fY, pTargetSLPoints->pSLPoint[0].iId);
	return 0;
}

// 求本车当前点在上一帧规划中的位置
int iFindHisPlanId(StrLocationFusion *pStrCarStatus, MapPoint *ReferencePoint)
{
	int i = 0;
	double deltMins0 = 0.0;
	double deltMins1 = 0.0;
	double delts = 0.0;
	int iId = 0;
	int iId0 = 0;
	int iId1 = 1;
	deltMins0 = sqrt((pStrCarStatus->fX - ReferencePoint[0].dX)* (pStrCarStatus->fX - ReferencePoint[0].dX) + (pStrCarStatus->fY - ReferencePoint[0].dY) * (pStrCarStatus->fY - ReferencePoint[0].dY) + (pStrCarStatus->fZ - ReferencePoint[0].dZ) * (pStrCarStatus->fZ - ReferencePoint[0].dZ));
	deltMins1 = sqrt((pStrCarStatus->fX - ReferencePoint[1].dX)* (pStrCarStatus->fX - ReferencePoint[1].dX) + (pStrCarStatus->fY - ReferencePoint[1].dY) * (pStrCarStatus->fY - ReferencePoint[1].dY) + (pStrCarStatus->fZ - ReferencePoint[1].dZ) * (pStrCarStatus->fZ - ReferencePoint[1].dZ));
	if (deltMins0 > deltMins1)
	{
		delts = deltMins0;
		deltMins0 = deltMins1;
		deltMins1 = delts;
		iId0 = 1;
		iId1 = 0;
	}

	for (i = 2; i < 50; i++)
	{
		delts = sqrt((pStrCarStatus->fX - ReferencePoint[i].dX)* (pStrCarStatus->fX - ReferencePoint[i].dX) + (pStrCarStatus->fY - ReferencePoint[i].dY) * (pStrCarStatus->fY - ReferencePoint[i].dY) + (pStrCarStatus->fZ - ReferencePoint[i].dZ) * (pStrCarStatus->fZ - ReferencePoint[i].dZ));
		if (delts < deltMins0)
		{
			deltMins1 = deltMins0;
			iId1 = iId0;
			deltMins0 = delts;
			iId0 = i;
		}
		else if (delts > deltMins0 && delts < deltMins1)
		{
			deltMins1 = delts;
			iId1 = i;
		}
	}

	if (iId0 > iId1)
		iId = iId1;
	else
		iId = iId0;

	return iId;
}

// 计算本车当前位置与参考线的偏差
float fAngle(SLPoint *pVehicleSLPoint, int iId, StrSLPoints *pStrSLReferLines)
{
	float fAngle = 0.0;
	float fX1 = pVehicleSLPoint->fX;
	float fY1 = pVehicleSLPoint->fY;
	float fX2 = pStrSLReferLines->pSLPoint[iId].fX;
	float fY2 = pStrSLReferLines->pSLPoint[iId].fY;
	float fX3 = 0.0;
	float fY3 = 0.0;

	if (iId == pStrSLReferLines->iNum - 1)
	{
		fX3 = pStrSLReferLines->pSLPoint[iId].fX;
		fY3 = pStrSLReferLines->pSLPoint[iId].fY;
	}
	else
	{
		fX3 = pStrSLReferLines->pSLPoint[iId + 1].fX;
		fY3 = pStrSLReferLines->pSLPoint[iId + 1].fY;
	}

	fAngle = acos((fX1 *(fX3 - fX2) + fY1 * (fY3 - fY2)) / sqrt(fX1 * fX1 + fY1 * fY1) * sqrt((fX3 - fX2) * (fX3 - fX2) + (fY3 - fY2) * (fY3 - fY2))) * 180 / PI;

	return fAngle;
}

Point Enu2Rfu1(StrLocationFusion *pStrCarStatus, MapPoint *MapGpsPoint)
{
	Point ReturnPoint;
	double deltx = MapGpsPoint->dX - pStrCarStatus->fX;
	double delty = MapGpsPoint->dY - pStrCarStatus->fY;
	double deltz = MapGpsPoint->dZ - pStrCarStatus->fZ;
	Point TempPoint;
	double pitch = (pStrCarStatus->pitch * PI / 180);
	double roll = (pStrCarStatus->roll * PI / 180);
	double yaw = (pStrCarStatus->yaw *PI / 180);
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

int UltrasonicDecison(StrUltrasonicDatas *UltrasonicDatas, StrPlanningConfig *pStrPlanningConfig, EmDecision *emDecision, SLPoint *DecisionPoint)
{
	int i = 0;

	if (false == UltrasonicDatas->byHaveUltrasonic)
	{
		if (debug)
			printf("超声波没有检测到障碍物，超声波决策为:前进 \n");
		*emDecision = Forward;
		return -1;
	}

	if ((UltrasonicDatas->UltrasonicData[1].fDis <= 2.0 && UltrasonicDatas->UltrasonicData[1].fDis > 0.0 && UltrasonicDatas->UltrasonicData[1].UltrasonicAvailable == true) || (UltrasonicDatas->UltrasonicData[3].fDis <= 2.0 && UltrasonicDatas->UltrasonicData[3].fDis > 0.0 &&UltrasonicDatas->UltrasonicData[3].UltrasonicAvailable == true)) //左前or右前
	{
		if (UltrasonicDatas->UltrasonicData[1].fDis <= 2.0 && UltrasonicDatas->UltrasonicData[1].fDis > 0.0 && UltrasonicDatas->UltrasonicData[1].UltrasonicAvailable == true)
		{
			if (debug)
				printf("前左(1号)超声波检测到障碍物，距离为%f(小于2米)，超声波决策为:安全停车 \n", UltrasonicDatas->UltrasonicData[1].fDis);
		}
		else
		{
			if (debug)
				printf("前右(3号)超声波检测到障碍物，距离为%f(小于2米)，超声波决策为:安全停车 \n", UltrasonicDatas->UltrasonicData[3].fDis);
		}
		*emDecision = Safestop;
		return 0;
	}

	for (i = 0; i < ULTRASOUND_NUM; i++)
	{
		if (UltrasonicDatas->UltrasonicData[i].fDis <= 0.5 && UltrasonicDatas->UltrasonicData[i].fDis > 0.0 && UltrasonicDatas->UltrasonicData[i].UltrasonicAvailable == true)
		{
			if (debug)
				printf("%d号超声波检测到障碍物，距离为%f(小于0.5米)，超声波决策为:安全停车 \n", i, UltrasonicDatas->UltrasonicData[i].fDis);
			*emDecision = Safestop;
			return 0;
		}
	}

	if (UltrasonicDatas->UltrasonicData[0].UltrasonicAvailable == false && UltrasonicDatas->UltrasonicData[2].UltrasonicAvailable == false)
	{
		if (debug)
			printf("左前(0号)和右前(2号)超声波都没检测到障碍物，超声波决策为:前进 \n");
		*emDecision = Forward;
		return -1;
	}

	if ((UltrasonicDatas->UltrasonicData[0].fDis > 0.5 && UltrasonicDatas->UltrasonicData[0].UltrasonicAvailable == true && UltrasonicDatas->UltrasonicData[0].fDis < pStrPlanningConfig->fVehicleWidth) && ((UltrasonicDatas->UltrasonicData[2].fDis > pStrPlanningConfig->fVehicleWidth && UltrasonicDatas->UltrasonicData[2].UltrasonicAvailable == true) || UltrasonicDatas->UltrasonicData[2].UltrasonicAvailable == false))
	{
		if (debug)
			printf("0号超声波检测到障碍物距离在[0.5,%f]区间范围内，将该障碍物作为决策点 \n", pStrPlanningConfig->fVehicleWidth);
		DecisionPoint->fS = 1.5;
		DecisionPoint->fL = VehicleSLPoint.fL + 0.5 * pStrPlanningConfig->fVehicleWidth + UltrasonicDatas->UltrasonicData[0].fDis;
		DecisionPoint->fTheta = 0.0;
		*emDecision = RightPass;
		if (debug)
			printf("超声波决策为:往障碍物的右侧绕行，决策点的SL坐标为:S=%f, L=%f \n", DecisionPoint->fS, DecisionPoint->fL);
	}
	else if ((UltrasonicDatas->UltrasonicData[2].fDis > 0.5 && UltrasonicDatas->UltrasonicData[2].UltrasonicAvailable == true && UltrasonicDatas->UltrasonicData[2].fDis < pStrPlanningConfig->fVehicleWidth) && ((UltrasonicDatas->UltrasonicData[0].fDis > pStrPlanningConfig->fVehicleWidth && UltrasonicDatas->UltrasonicData[0].UltrasonicAvailable == true) || UltrasonicDatas->UltrasonicData[0].UltrasonicAvailable == false))
	{
		if (debug)
			printf("2号超声波检测到障碍物距离在[0.5,%f]区间范围内，将该障碍物作为决策点 \n", pStrPlanningConfig->fVehicleWidth);
		DecisionPoint->fS = 1.5;
		DecisionPoint->fL = VehicleSLPoint.fL - 0.5 * pStrPlanningConfig->fVehicleWidth - UltrasonicDatas->UltrasonicData[2].fDis;
		DecisionPoint->fTheta = 0.0;
		*emDecision = LeftPass;
		if (debug)
			printf("超声波决策为:往障碍物的左侧绕行，决策点的SL坐标为:S=%f, L=%f \n", DecisionPoint->fS, DecisionPoint->fL);
	}
	else if ((UltrasonicDatas->UltrasonicData[0].UltrasonicAvailable == true && UltrasonicDatas->UltrasonicData[0].fDis > 0.5 && UltrasonicDatas->UltrasonicData[0].fDis < pStrPlanningConfig->fVehicleWidth) && (UltrasonicDatas->UltrasonicData[2].UltrasonicAvailable == true && UltrasonicDatas->UltrasonicData[2].fDis > 0.5 && UltrasonicDatas->UltrasonicData[2].fDis < pStrPlanningConfig->fVehicleWidth))
	{
		if (debug)
			printf("0号(左前)超声波:障碍物与车相距%f米，2号(右前)超声波:障碍物与车相距%f米，超声波决策为：安全停车 \n", UltrasonicDatas->UltrasonicData[0].fDis, UltrasonicDatas->UltrasonicData[2].fDis);
		*emDecision = Safestop;
	}

	return 0;
}

// 规划轨迹中点的数量(pStrRoutePaths->strRoutePath[i].iNum = iOutNum)一般不是50
// (1) 如果上一帧规划失败，则本次规划轨迹点的总数iOutNum等于离规划目标点最近参考点的索引号。
// 规划点的集合从规划的起始点（本车最近的参考点）开始，直到规划目标点iOutNum的取值区间为[0,50)，取值一般不是49，通常会远远小于50，例如，可能会等于24
// (2) 如果上一帧规划成功，则本次规划轨迹的点的总数iOutNum等于离规划目标点最近参考点的索引号-2。
// 规划点的集合从规划的起始点（HisPlanPoints.pSLPoint[HisId + 2]这个点）开始，直到规划目标点
int iRoutePlanner(StrSLPoints *pStrSLReferLines, StrLocationFusion *pStrCarStatus, StrTargetFusion *pTargetsFusion, StrPlanningConfig *pStrPlanningConfig, StrRoutePaths *pStrRoutePaths)
{
	int i = 0;
	int j = 0;
	int k = 0;
	int iRet = 0;

	if (NULL == pStrSLReferLines || NULL == pStrCarStatus || NULL == pTargetsFusion || NULL == pStrPlanningConfig)
	{
		if (debug)
			printf("Input Param Error\n");
		return -1;
	}

	if (NULL == pStrRoutePaths)
	{
		if (debug)
			printf("Output Param Error\n");
		return -1;
	}

	// second：将本车位置投影到参考路径上，求出对应的S，L以及航向
	StrSLPoints TargetSLPoints;
	double dx0 = pStrSLReferLines->pSLPoint[0].fX;
	double dy0 = pStrSLReferLines->pSLPoint[0].fY;
	double dx1 = pStrSLReferLines->pSLPoint[1].fX;
	double dy1 = pStrSLReferLines->pSLPoint[1].fY;
	VehicleSLPoint.fX = 0.0;	// 本车坐标系原点
	VehicleSLPoint.fY = 0.0;
	VehicleSLPoint.fS = fabs((dx0 *(dx1 - dx0) + dy0 * (dy1 - dy0)) / sqrt((dx1 - dx0) * (dx1 - dx0) + (dy1 - dy0) * (dy1 - dy0)));
	float fL = sqrt(dx0 * dx0 + dy0 * dy0 - VehicleSLPoint.fS * VehicleSLPoint.fS);
	float fCos = (dx1 - dx0) / sqrt((dx1 - dx0) * (dx1 - dx0) + (dy1 - dy0) * (dy1 - dy0));
	float fSin = (dy1 - dy0) / sqrt((dx1 - dx0) * (dx1 - dx0) + (dy1 - dy0) * (dy1 - dy0));
	float f = dx0 * fSin - dy0 * fCos;
	if (f < 0)
		VehicleSLPoint.fL = -fL;
	else
		VehicleSLPoint.fL = fL;
	VehicleSLPoint.fTheta = 0.0;
	VehicleSLPoint.fV = pStrCarStatus->fSpeed;
	StrTargetFusion RadarTargetsFilterFusion;

	// 超声波决策
	UltrasonicDecison(&pTargetsFusion->strUltrasonicDatas, pStrPlanningConfig, &UltrasonicDecision, &UltrasonicDecisionPoint);
	// 毫米波过滤
	iRadarTargetFilter(pTargetsFusion, pStrSLReferLines, &RadarDecision, &RadarTargetsFilterFusion);

	if (UltrasonicDecision != Safestop && RadarDecision != Safestop)
		Decision = iTargetFilter(&RadarTargetsFilterFusion, pStrSLReferLines, pStrPlanningConfig, &DecisionPoint);
	else
	{
		Decision = Safestop;
		if (UltrasonicDecision == Safestop)
		{
			if (debug)
				printf("超声波决策为：安全停车\n");
		}
		if (RadarDecision == Safestop)
		{
			if (debug)
				printf("毫米波决策为：安全停车\n");
		}
		UltrasonicDecision = Forward;
	}

	if (debug)
		printf("决策点的SL坐标是：S = %f，L = %f \n", DecisionPoint.fS, DecisionPoint.fL);
	if (debug)
		printf("本车的SL坐标是：S = %f，L = %f \n", VehicleSLPoint.fS, VehicleSLPoint.fL);

	if (Decision == Safestop)
		SafestopDecision = Safestop;

	if (SafestopDecision == Safestop)
	{
		if (Decision == Safestop)
			SafeStopNum = 0;
		else
		{
			SafeStopNum++;
			if (SafeStopNum < 40)
			{
				Decision = Safestop;
				if (debug)
					printf("安全停车计数为： %d\n", SafeStopNum);
			}
			else
				SafestopDecision = Decision;
		}
	}

	if (Decision == Forward)
	{
		if (debug)
			printf("整体决策为：前进\n");
	}
	else if (Decision == Follow)
	{
		if (debug)
			printf("整体决策为：跟车\n");
	}
	else if (Decision == LeftPass)
	{
		if (debug)
			printf("整体决策为：左侧绕行\n");
	}
	else if (Decision == RightPass)
	{
		if (debug)
			printf("整体决策为：右侧绕行\n");
	}
	else if (Decision == Safestop)
	{
		if (debug)
			printf("整体决策为：安全停车\n");
	}
	else if (Decision == Quit)
	{
		if (debug)
			printf("整体决策为:退出\n");
	}

	if (Safestop == Decision)
		return 1;

	HisDecision = Decision;
	// third：根据决策输出结果，计算目标的S,L和航向角
	iRet = iTargetSelect(&VehicleSLPoint, pTargetsFusion, pStrSLReferLines, &Decision, &DecisionPoint, pStrPlanningConfig, &TargetSLPoints);
	if (-1 == iRet)
	{
		if (debug)
			printf("目标选择错误 \n");
        usleep(1000000);
		return -1;
	}

	TargetPoint = TargetSLPoints.pSLPoint[0];
	//VehiclePoint = VehicleSLPoint;
	for (i = 0; i < TargetSLPoints.iNum; i++)
	{
		if (debug)
			printf("规划目标点的坐标：X=%f，Y=%f，S=%f，L=%f，Id=%d \n", TargetSLPoints.pSLPoint[i].fX, TargetSLPoints.pSLPoint[i].fY, TargetSLPoints.pSLPoint[i].fS, TargetSLPoints.pSLPoint[i].fL, TargetSLPoints.pSLPoint[i].iId);
	}

	if (1 == PlanStatus)
	{
		// 规划起始点赋值
		PlanStartPoint.fX = HisPlanPoints.pSLPoint[HisId + 2].fX;
		PlanStartPoint.fY = HisPlanPoints.pSLPoint[HisId + 2].fY;
		PlanStartPoint.fZ = HisPlanPoints.pSLPoint[HisId + 2].fZ;
		PlanStartPoint.fS = HisStrSLReferLine.pSLPoint[HisId + 2].fS - HisStrSLReferLine.pSLPoint[HisId].fS;
		PlanStartPoint.fL = HisPlanPoints.pSLPoint[HisId + 2].fL;
		PlanStartPoint.fTheta = HisPlanPoints.pSLPoint[HisId + 2].fTheta;
		if (PlanStartPoint.fS < -10.00)
		{
			if (debug)
				printf("警告：本次规划起始点的S坐标小于-10米 \n");
		}
		if (debug)
			printf("规划起始点的坐标：X=%f，Y=%f，S=%f，L=%f，theta=%f\n", PlanStartPoint.fX, PlanStartPoint.fY, PlanStartPoint.fS, PlanStartPoint.fL, PlanStartPoint.fTheta);
		HisPlanPoints.pSLPoint[0].fS = HisStrSLReferLine.pSLPoint[HisId].fS - HisStrSLReferLine.pSLPoint[HisId].fS; // 等于0
		HisPlanPoints.pSLPoint[1].fS = HisStrSLReferLine.pSLPoint[HisId + 1].fS - HisStrSLReferLine.pSLPoint[HisId].fS;
		HisPlanPoints.pSLPoint[2].fS = HisStrSLReferLine.pSLPoint[HisId + 2].fS - HisStrSLReferLine.pSLPoint[HisId].fS;
	}

	pStrRoutePaths->iRouteNum = 1;	// 输出路径数
	float pfL[MAX_SLPOINT_NUM];
	int iOutNum = 0;
	float pfC[4];
	int iNum = 0;
	float fStartS = 0.0;
	float fSideDisStart = 0.0;
	float fEndS = 0.0;
	float fSideDisEnd = 0.0;
	float fThetaStart = 0.0;
	float fThetaEnd = 0.0;
	int iId = 0;

	// fourth:求解三次曲线
	for (i = 0; i < TargetSLPoints.iNum; i++)
	{
		if (1 == PlanStatus)
		{
			// 规划的起点赋值
			fStartS = PlanStartPoint.fS;
			fSideDisStart = PlanStartPoint.fL;
			fThetaStart = PlanStartPoint.fTheta;
			// 规划的目标点赋值
			// 在进行参数拟合时，为了统一SL坐标值，SL坐标系是以上一帧规划点序列HisPlanPoints中的HisId这个点作为坐标原点
			// 然而，TargetSLPoints.pSLPoint[0]这个点的SL坐标是在本次SL坐标系下计算得来的。所以，在计算规划目标点在统一坐标系下的S坐标时，需要加上起始点的S坐标值
			TargetSLPoints.pSLPoint[0].fS += PlanStartPoint.fS;		// 注意，规划点的S坐标比原来增加了PlanStartPoint.fS
			fEndS = TargetSLPoints.pSLPoint[0].fS;
			fSideDisEnd = TargetSLPoints.pSLPoint[0].fL;
			fThetaEnd = 0.0;
			// 如果上一帧规划成功，则本次规划轨迹的点的总数iOutNum等于离规划目标点最近参考点的索引号-2
			// 规划点的集合从规划的起始点（HisPlanPoints.pSLPoint[HisId + 2]这个点）开始，直到规划目标点
			iId = 3;
			iOutNum = iFindNearestS(&TargetSLPoints.pSLPoint[0], pStrSLReferLines) - 2;
		}
		else
		{
			// 规划的起点赋值
			fStartS = VehicleSLPoint.fS;
			fSideDisStart = VehicleSLPoint.fL;
			fThetaStart = 0.0;
			// 规划的目标点赋值
			fEndS = TargetSLPoints.pSLPoint[0].fS;
			fSideDisEnd = TargetSLPoints.pSLPoint[0].fL;
			fThetaEnd = 0.0;
			// 如果上一帧规划失败，则本次规划轨迹点的总数iOutNum等于离规划目标点最近参考点的索引号
			// 规划点的集合从规划的起始点（本车最近的参考点）开始，直到规划目标点
			// iOutNum的取值区间为[0,50)，取值一般不是49，通常会远远小于50，例如，可能会等于24
			iId = 1;
			iOutNum = iFindNearestS(&TargetSLPoints.pSLPoint[0], pStrSLReferLines);
		}
		if (debug)
			printf("本次规划的点数为:%d \n", iOutNum);

		// 求解三次曲线系数
		if (debug)
			printf("规划的起始位置：S=%f，L=%f，theta=%f \n", fStartS, fSideDisStart, fThetaStart);
		if (debug)
			printf("规划的目标位置：S=%f，L=%f，theta=%f \n", fEndS, fSideDisEnd, fThetaEnd);
		vParasSolver(fStartS, fEndS, fSideDisStart, fSideDisEnd, fThetaStart, fThetaEnd, pfC, &iNum);

		// 计算pStrSLReferLines中位于[iId, iOutNum+iId) 区间范围内的参考点的L坐标值
		// 当PlanStatus为1时，区间为[3,规划目标点最近的索引号];  当PlanStatus为0时，区间为[1,规划目标点最近的索引号];
		// 需要注意的是，规划目标点的S坐标与障碍物的S坐标大小相近，三阶多项式的参数是使用规划起点和目标点拟合计算出来的
		vCal_l(pfC, iNum, pStrSLReferLines, iId, pfL, iOutNum);
		pStrRoutePaths->strRoutePath[i].iNum = iOutNum;

		// 三次曲线系数赋值
		for (j = 0; j < iNum; j++)
			pStrRoutePaths->strRoutePath[i].pfC[j] = pfC[j];
		if (debug)
			printf("三次多项式的系数为：%f，%f，%f，%f \n", pStrRoutePaths->strRoutePath[i].pfC[0], pStrRoutePaths->strRoutePath[i].pfC[1], pStrRoutePaths->strRoutePath[i].pfC[2], pStrRoutePaths->strRoutePath[i].pfC[3]);

		for (j = 0; j < iOutNum; j++)
		{
			// 三次曲线侧向偏差
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].iId = j;
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fL = pfL[j];
			pStrRoutePaths->strRoutePath[i].pSLPoint[j].fTheta = atan(fLSolver(pfC, pStrSLReferLines->pSLPoint[j + iId].fS, 1));
		}

		// 把pStrSLReferLines中参考点的XYZ坐标值和S坐标值赋给pStrRoutePaths->strRoutePath[0]中相应的轨迹点
		iCurtoCar(pStrSLReferLines, pfL, iId, iOutNum, &pStrRoutePaths->strRoutePath[i]);
	}

	return 0;
}


