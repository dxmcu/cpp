#ifndef __TYPE_DEFINE_H__
#define __TYPE_DEFINE_H__

#include <Eigen/Dense>
using namespace Eigen;

typedef unsigned char    uint8;
typedef unsigned short   WORD;
//typedef unsigned int     DWORD;
//typedef unsigned long    ULONG32;
typedef unsigned long long  ULONG64;
typedef unsigned short		Bool;						/* boolean */
typedef void            * Ptr;							/* data pointer */
typedef long long Int64;		
///< Signed 64-bit integer

#define ULTRASOUND_NUM 8

typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define DEBUG

#define Log_error(x, ...)  do { printf("\n\r [Auto_Car_ERROR] %s:%s:%d:" x, __FILE__, __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while(0)
#define Log_infor(x, ...) do { printf("\n\r [Auto_Car_INFO] %s:%s:%d:" x, __FILE__, __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while(0)
#define Log_inforcolor(x, ...) do { printf("\033[1m\033[41;37m [INFO]  %s:%s:%d: " x"\033[0m", __FILE__, __func__, __LINE__, ##__VA_ARGS__); fflush(stdout); } while(0)
#define Log_debug printf
#define Log_print										//printf

#define MAX_SLPOINT_NUM 100
#define MAX_PLANNER_ROUTE_NUM 10						//最大规划路径数

#define MAX_TARGET_NUM 500
#define EDGE_NUM   4
#define MAX_ROAD_BOARD_NUM 100

#define PI 3.1415926535898
#define EQUAL 0.000001
#define MINR 6.75
#define MAXKAPPA 0.1538

//Planning Module Param Struct
typedef struct Struct_PlanningConfig
{
  double fMinSafeDis;									//纵向最小安全距离
  double fMaxPlanLength;								//最大规划距离
  double fMaxAcc;										//最大加速度
  double fMaxV;										//最大速度
  double follow_dis;									//跟车距离
  double fUltrasonicSpeed;
  double fUltrasonicDis;
  double fVechicleWidth;								//车身宽度
  double fVechicleLength;								//车身长度
  double fSampleBuffer;
  double fSampleAccuracy;
  double fSampleTime;
  double fUnitS;
  double fl0;
  double fb;
  double fk;
  double fPathLCost;
  double fPathDLCost;
  double fPathDDLCost;
  double fEndCost;
  double fNudgeBuffer;
  double fIgnoreL;
  double fObstacleCollisionCost;
  double fObstacleCollisionDis;
  double fObstacleBuffer;
  double fSafeDis;
  double fRoadWidth;   //道路宽度默认值
  double dOriginLat;  //ENU坐标系原点定义
  double dOriginLon;
  double dOriginAlt;
  int iRecordFile;
}StrPlanningConfig;


//Control Module Param Struct
typedef struct Struct_ControllingConfig
{
  double ThrottlePushValue_high;  		//high speed throttle push rod value
  double ThrottlePushValue_low;	  	//low speed throttle push rod value
  double Swith_acc;     				//Acceleration switch value
  double Kp;       					//PID control coefficient Kp
  double Ki;  						//PID control coefficient Ki
  double Kd;  						//PID control coefficient Kd
  double Brake_speed;  				//slow down brake value
  double Brake_emergency;     		//Emergency brake value
  double dt;   						//Lateral control discrete time
  int    Enable_Saturationintegrator;	//Integral restriction
  double f_saturationintegrator_high;	//Upper limit of saturation integral
  double f_saturationintegrator_low;		//Lower limit of saturation integral
  double fmaxhingeangle;				//Maximum hinged angle
  double ftolerance;					//Solving maximum tolerance
  int    max_num_iteration;			//Solving maximum iteration number
  double x_polearm;					//pole arm vector in X axis direction
  double y_polearm;					//pole arm vector in Y axis direction
  double z_polearm;					//pole arm vector in Z axis direction
  double lat_err_upper;               //Upper limit of lateral error
  double lat_err_lower;               //Lower limit of lateral error
  double lat_speed_err_upper;          //Upper limit of lateral velocity error
  double lat_speed_err_lower;         //Lower limit of lateral velocity error
  double BrakePushValue;            //Parking Brake Propulsion
}StrControllingConfig;


//SL坐标系的边界结构体定义
typedef struct Struct_SLBOUNDARY
{
  float fMaxS;
  float fMinS;
  float fMaxL;
  float fMinL;
}StrSLBoundary;

//高精地图数据结构体
typedef struct _Struct_Hd_Map
{
  int iId;
  double Lat;
  double Lon;
  double Alt;
  double Pitch;
  double Roll;
  double Yaw;
  double dX;
  double dY;
  double dZ;
}MapPoint;

typedef struct _Struct_Hd_Map_
{
  int    iNum;
  MapPoint *MapPoints;
}StrHdMap;

//Map Data Struct
typedef struct Point{									//点的数据结构
  uint8 ID;											// ID
  double X;											// X 本车坐标系的X
  double Y;											// Y本车坐标系的Y
  double Z;
}Point;

typedef struct Global
{
  Point  POINT[50];									// 高精度地图的点
  uint8  TYPE_ROAD;									// 出入弯点（分别代表什么？）
  uint8  PARK;										// 停车
  float  V;											// 速度（m/s,*100倍）
  double YAW;											// 航向角(degree)
  double L0;											// 车的纬度(degree)
  double B0;											// 车的经度(degree)
  int    NUM_POINT;									// 传输给局部的点的数量
}Global;

typedef struct _Struct_SL_Point
{
  float fS;											//本点对应位移
  float fL;											//本点对应侧向偏差
  float fDL;											//本点对应侧向偏差一阶导数
  float fDDL;											//本点对应侧向偏差二阶导数
  float fTheta;										//本点对应航向角
  float fKappa;
  float fV;											//本点速度
  float fA;											//本点加速度
  float fX;
  float fY;
  float fZ;
  int   iId;
}SLPoint;

typedef enum _Enum_Plan_Status
{
  QUIT_SYSTEM = -4,									//退出自动驾驶系统
  GLOAB_INVALID = -3,                                 //全局规划无效
  LOACTION_INVALID = -2,								//定位无效
  PLAN_FAILED = -1,									//规划失败
  PLAN_NORMAL = 0,									//正常规划
  ULTRASONIC_VALID = 1								//超声波有效
}EMPlanStatus;

typedef struct _Struct_SL_Points
{
  int iRoadType;
  int iNum;
  int iLightStatus; //转向灯状态：0：直行，1：左转，-1：右转
  int iSpoutWater;  //喷水状态：0：停止喷水，1：启动喷水
  int iCleanStatus; //清扫状态：0：停止清扫，1：启动清扫
  int iCleanAsh; //清灰状态：0：停止清灰,1：启动清灰
  int iStatus;         //-1异常状态:规划失败 -2:定位无效 0:正常规划状态 1:超声波雷达有效3:停车
  double pfC[4];
  SLPoint pSLPoint[MAX_SLPOINT_NUM];
}StrSLPoints;

typedef struct _Struct_Route_Points
{
  int iRouteNum;										//路径个数
  float fCost[MAX_PLANNER_ROUTE_NUM];
  StrSLPoints strRoutePath[MAX_PLANNER_ROUTE_NUM];	//每一条路径的信息
}StrRoutePaths;

typedef enum _Enum_Target_Type
{
  TARGET_INVALID = -1,
  TARGET_UNKNOW = 0,
  TARGET_VEHICLE,
  TARGET_PEDESTRIAN,
  TARGET_CYCLIST,
  TARGET_MOTORCYCLE,
}EmTargetType;

typedef enum _Enum_Sensor_Type
{
  SENSOR_UNKNOW = 0,
  SENSOR_RADAR,
  SENSOR_LIDAR,
  SENSOR_CAMERA,
  SENSOR_ULTRASONIC,
}EmSensorType;

typedef enum _enum_MOTION_STATUS
{
  MOTION_UNKNOW = -1,
  MOTION_OFF = 0,
  MOTION_ON,
}EmMotionStatus;

typedef struct _Struct_Point
{
  float fX;
  float fY;
  float fZ;
}StrPoint;

typedef struct _Struct_Target_Border
{
  unsigned char byHaveBorder;
  float fTargetAngle;
  StrPoint strTargetPoint[EDGE_NUM];
}StrTargetBoard;

//目标状态信息
typedef struct _Struct_Target_Status
{
  EmTargetType emTargetType;							//目标类型
  EmMotionStatus emMotionStatus;						//目标运动属性
  EmSensorType  emSensorType;
  int iTargetId;										//目标ID
  float fX;											//目标位置
  float fY;
  float fWidth;                                       //宽度
  float fLength;                                      //长度
  float fRate;										//目标速度（相对于本车）
  float fAngle;										//目标航向角
  float fAngleRate;									//目标角速度
  StrTargetBoard strTargetBoard;						//目标边界大小
}StrTargetStatus;

//道路边界信息
typedef struct _Struct_Road_Board
{
  int iRoadPointNum;
  StrPoint strLeftRoadBoard[MAX_ROAD_BOARD_NUM];
  StrPoint strRightRoadBoard[MAX_ROAD_BOARD_NUM];
}StrRodeBoard;

typedef struct _Struct_Ultrasonic_Data
{
  int iId;          //0-7
  bool UltrasonicAvailable;
  float fDis;
}StrUltrasonicData;
//超声波信息
typedef struct _Struct_Ultrasonic_Datas
{
  unsigned long long ullTimestamp;					//时间戳
  bool byHaveUltrasonic; //是否有超声波
  StrUltrasonicData UltrasonicData[ULTRASOUND_NUM];
}StrUltrasonicDatas;
#if 0
typedef struct _Struct_Ultrasonic_Data
{
  unsigned long long ullTimestamp;					//时间戳
  unsigned char byHaveUltrasonic;
  float fFL;  //前左 >=0有效 -1 无效
  float fF;   //前中
  float fFR;  //前右
  float fLF;  //左前
  float fLB;  //左后
  float fRF;  //右前
  float fRB;  //右后
  float fBL; //后左
  float fB;  //后中
  float fBR; //后右
}StrUltrasonicData;
#endif

//目标融合输出结构体
typedef struct _Struct_Target_Fusion
{
  unsigned long long ullTimestamp;	 //时间戳
  int iTargetNum;   //目标个数
  StrTargetStatus strTargetStatus [MAX_TARGET_NUM]; //目标状态信息
  StrRodeBoard strRoadBoard;   //道路边界信息
  StrUltrasonicDatas strUltrasonicDatas;  //超声波信息
}StrTargetFusion;

//定位融合输出结构体定义
typedef struct _Str_Location_Fusion_
{
  unsigned long long ullTimeStamp;					//时间戳
  double dLatitude;									//纬度
  double dLongitude;									//经度
  double dAltitude;									//高度
  double fX;
  double fY;
  double fZ;
  float fSpeed;										//本车当前速度
  float fAcceleration;								//本车当前加速度
  float fAngle;										//航向角
  float fAngleRate;									//本车当前角速度
  float fKappa;
  float velNorth;	//vehicle northward velocity
  float velEast; //vehicle eastward velocity
  float velUp;     //vehicle upward velocity
  float pitch;
  float roll;
  float yaw;
  float gyroX;   //vehicle X-axis angular velocity
  float gyroY;   //vehicle Y-axis angular velocity
  float gyroZ;    //vehicle Z-axis angular velocity
  char cSystemStatus;									//定位融合系统状态：可靠和不可靠或者未知
}StrLocationFusion;

//Trajectory Output
typedef struct Struct_Trajectory
{
  float fX;
  float fY;
  float fA;
  float fV;
  float fTheta;
  float fCurvature;
}StrTrajectory;

typedef struct Struct_Trajectorys
{
  int iNum;                      //轨迹点的个数
  StrTrajectory pStrTrajectory[100]; //轨迹点
}StrTrajectorys;

//Decision Result
typedef enum _Enum_Decision
{
  Quit = -1,      //安全或者人为原因退出自动驾驶模式
  ForWard = 0,   //跟随默认轨迹
  Pass,         //避障前进
  Follow,      //跟随同方向障碍物
  Safestop,   //停止前进
}EmDecision;


#endif
