#pragma once
#include <map>
#include <jsoncpp/json/json.h>
#include "common_defs.h"
#include "TrackObject.h"
#include "LossMat.h"
#include "SensorData.h"
#include "KalmanFilter.h"
using namespace std;
class CTracker
{
public:
	CTracker();
	~CTracker();
	/*
	* \brief 获取最新的ID
	*/
	int GetNewId(){ return id_count++; };
	
	/*
	* \brief 增加新的跟踪对象
	* @param obj  新增的跟踪对象
	* @result  新增对象的id
	*/
	int PushTrackObject(TrackedObjectPtr obj);

	/*
	* \brief 执行障碍物跟踪
	* @param[IN] UpdatedSense  新感知的障碍物结果
	*/
	bool Track(UpdateTargetData&  update_sersor_data);

	/*
	* \brief 处理收到的数据，转换成TRACK需要的格式
	* param[in] RecvGPS , 本车位置融合信息
	* param[in] UpdatedSense 障碍物数据
	* param[out] update_sersor_data  预备好的数据
	*/
    void PrepareData(Json::Value RecvGPS, Json::Value UpdatedSense , UpdateTargetData&  update_sersor_data);

	/*
	* \brief 单独进行激光雷达跟踪
	*/
	bool TrackLidar(UpdateTargetData* sensor_data);

	/*
	* \brief 单独进行毫米波雷达跟踪
	*/
	bool TrackRadar(UpdateTargetData* sensor_data);

	/*
	* \brief 把StrStructStatus 数据结构包装成一个共享指针结构
	* @param update_status 当前获取的一个障碍物信息结构体
	* @return 包装的SensorData Share 指针
	*/
	UpdateObjectPtr WrapSensorData(StrTargetStatus* update_status);

	void CollectSensorData(std::vector<UpdateObjectPtr>* new_objects, UpdateTargetData* sensor_data, EmSensorType sensor_type);

	/*
	* \brief 计算最新获取的障碍物感知对象和跟踪列表内对象的损失值
	* @param tracked 已跟踪的障碍物目标
	* @param new_object 新探测到的障碍物目标
	*/
	float ComputeDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object,  int loss_type);
	
	/*
	* \brief 计算待匹配目标中心点之间距离损失
	* @param tracked 已跟踪的障碍物目标
	* @param new_object 新检测到的障碍物目标
	* @return 返回损失值
	*/
	float ComputeLocationDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object);
	
	/*
	* \brief 计算待匹配目标角点之间的距离损失的平均
	* @param tracked 已跟踪的障碍物目标
	* @param new_object 新检测到的障碍物目标
	* @return 返回损失值的平均
	*/
	float ComputeCornersDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object);

	/*
	* \brief 计算待匹配目标IOU损失，暂时简单地用两个对象的外接框来处理
	* @param tracked 已跟踪的障碍物目标
	* @param new_object 新检测到的障碍物目标
	* @return 返回损失值
	*/
	float ComputeIOUDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object);
	/*
	* \brief 初始化跟踪列表
	* \param[IN] pTargetsFusion  当前检测障碍物
	*/
	void InitTrackObjects(UpdateTargetData *pUpdateSensorData);

	/*
	* \brief 将融合的结果打包成PLANNING模块接收的jason格式
	* paran[out]  融合后的JSON数据包 
	*/
	void PackFusionObjects(Json::Value& funsion_sense);

	/*
	* \brief 执行匈牙利匹配
	* \param[IN] loss_matrix 执行匈牙利匹配的损失矩阵
	* \param[IN] assignments 匹配关系
	* \param[IN] unassigned_rows  行方向未匹配，已跟踪目标未匹配到新目标
	* \param[IN] unassigned_cols  列方向未匹配，新目标未匹配到老目标
	*/
	void Match(LossMat<float>* cost_matrix, std::vector<std::pair<size_t, size_t>>* assignments,
		std::vector<size_t>* unassigned_rows,
		std::vector<size_t>* unassigned_cols);

	/*
	*  \brief 以下两个函数从PLANNING 模块复制，用来解析车辆状态数据和障碍物感知数据
	*         但是，其中将障碍物坐标从车体坐标转换到本地坐标(ENU),以便进行全局跟踪
	*/
	int iJsonToTargetsStatus(Json::Value RecvSense, UpdateTargetData *pUpdateSersorData);
	int iJsonToCarStatus(Json::Value RecvGPS, StrLocationFusion *pStrCarStatus);

	/*
	*  \brief 将融合结果转换成JSON格式
	*  param[in] 待转换的JSON变量
	*  RETURN:  成功与否标志
	*/
	bool iTargetsStatusToJson(Json::Value& RecvSense);

	/*
	*  \brief 判断某个障碍物是否和跟踪列表内障碍物有交集,如果没有就新增，如果有就更新跟踪对象尺寸
	*  param[in] obj 检测障碍物
	*  RETURN:  void
	*/
    void UpdateSizeOrAddNewObject(TrackedObjectPtr obj);
protected:
	const unsigned long long INVISBLE_LIMIT = 300;

	//kalman滤波器
	std::shared_ptr<CKalmanFilter> kalman_filter;

	//跟踪初始化标志
	bool b_initialized;

	//用来生成跟踪对象ID
	int id_count;
	
	//指最近一次更新跟踪列表时的车辆状态
	StrLocationFusion latest_vehicle_location;

	//跟踪目标列表
	std::vector<TrackedObjectPtr> tracked_lidar_objects_;
	std::vector<TrackedObjectPtr> tracked_radar_objects_;

	//IOU损失的阈值控制值
	double background_object_match_threshold_ = 4.0;
	float max_match_distance_ = 4.0f;

	std::vector<int> row_tag_;
	std::vector<int> col_tag_;
	
	// 预测变量
	mutable MlfPredict predict_;

protected:
	//ENU坐标原点
	MapPoint OriginPoint;

};

