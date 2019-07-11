#include "Tracker.h"
#include <iostream>
#include <time.h>
#include "TrackObject.h"
#include "MatchCost.h"
//#include "gated_hungarian_bigraph_matcher.h"
#include "VectorUtil.h"
#include "GeoUtil.h"
CTracker::CTracker()
{
	b_initialized = false;
	kalman_filter = std::make_shared<CKalmanFilter>();
	kalman_filter->InitParameters(0.01);

	OriginPoint.Lat = 31.815585704117336;
	OriginPoint.Lon = 120.01273315260624;
	OriginPoint.Alt = 0.0;

}
CTracker::~CTracker()
{

}
int CTracker::PushTrackObject(TrackedObjectPtr obj)
{
	int tid = GetNewId();
	obj->target_status.iTargetId = tid;
	obj->latest_anchor_point(0) = obj->target_status.fX;
	obj->latest_anchor_point(1) = obj->target_status.fY;

	//记录最近跟踪时刻的车辆信息
	obj->latest_vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
	obj->latest_vehicle_angle = latest_vehicle_location.yaw - 90;

	if (obj->target_status.emSensorType == SENSOR_LIDAR)
	{
		obj->latest_velocity(0) = 0.0;
		obj->latest_velocity(1) = 0.0;
		tracked_lidar_objects_.push_back(obj);
	}
	if (obj->target_status.emSensorType == SENSOR_RADAR)
	{
		double angle = obj->target_status.fAngle;// atan(obj->target_status.fY / obj->target_status.fX);
		obj->latest_velocity(0) = obj->target_status.fRate*cos(angle);
		obj->latest_velocity(1) = obj->target_status.fRate*sin(angle);
		tracked_radar_objects_.push_back(obj);
	}

	return tid;
}
void CTracker::UpdateSizeOrAddNewObject(TrackedObjectPtr obj)
{
	if (obj->target_status.emSensorType == SENSOR_LIDAR)
	{
		//tracked_lidar_objects_.push_back(obj);
	}
	if (obj->target_status.emSensorType == SENSOR_RADAR)
	{
		for (int i = 0; i < tracked_radar_objects_.size(); i++)
		{
			if (tracked_radar_objects_[i]->Close2Update(obj->target_status, tracked_radar_objects_[i]->latest_vehicle_pos, tracked_radar_objects_[i]->latest_vehicle_angle))
			{
				//记录最近跟踪时刻的车辆信息
				tracked_radar_objects_[i]->latest_vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
				tracked_radar_objects_[i]->latest_vehicle_angle = latest_vehicle_location.yaw - 90;
				return;
			}

		}
		PushTrackObject(obj);
	}
	return ;
}
void CTracker::InitTrackObjects(UpdateTargetData *pUpdateSensorData)
{	
	StrTargetFusion* pTargetsFusion = &(pUpdateSensorData->target_data);
	for (int i = 0; i < pTargetsFusion->iTargetNum; i++)
	{
			TrackedObjectPtr new_object = std::make_shared<CTrackObject>();
			memcpy(&(new_object->target_status), &(pTargetsFusion->strTargetStatus[i]), sizeof(StrTargetStatus));
			new_object->start_time = pTargetsFusion->ullTimestamp;
			new_object->latest_time = pTargetsFusion->ullTimestamp;
		
			PushTrackObject(new_object);
	}
}

bool CTracker::TrackRadar(UpdateTargetData* update_sersor_data)
{
	//这里开始障碍物跟踪
	//1. 初始化损失矩阵
	LossMat<float> loss_mat;
	loss_mat.Resize(tracked_radar_objects_.size(), update_sersor_data->radar_count);
	row_tag_.assign(loss_mat.height(), 0);
	col_tag_.assign(loss_mat.width(), 0);
	std::vector<UpdateObjectPtr> new_radar_objects;
	CollectSensorData(&new_radar_objects, update_sersor_data, EmSensorType::SENSOR_RADAR);

	//2.计算radar损失矩阵
	for (int i = 0; i < tracked_radar_objects_.size(); i++)
	{
		for (int j = 0; j < new_radar_objects.size(); j++)
		{
			UpdateObjectPtr new_object = new_radar_objects[j];
			//设置当前车辆位置
			new_object->vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
			new_object->vehicle_angle = latest_vehicle_location.yaw - 90;
			tracked_radar_objects_[i]->PredictState(update_sersor_data->target_data.ullTimestamp);

			loss_mat(i, j) = ComputeDistance(tracked_radar_objects_[i],
				new_object,
				Enum_DISTANCE_LOSS_TYPE::Center);

		}
		//刷新track object的跟踪时间
		tracked_radar_objects_[i]->latest_time = update_sersor_data->target_data.ullTimestamp;
	}

	//3. 执行GNN匹配
	std::vector<std::pair<size_t, size_t>> assignments;
	std::vector<size_t> unassigned_rows;
	std::vector<size_t> unassigned_cols;
	Match(&loss_mat, &assignments, &unassigned_rows, &unassigned_cols);

	//4.给匹配对象赋分
	for (size_t i = 0; i < assignments.size(); ++i) {
		new_radar_objects[assignments.at(i).second]->association_score =
			loss_mat(assignments.at(i).first,
			assignments.at(i).second) / max_match_distance_;
	}
	//5.执行卡尔曼滤波
	for (size_t j = 0; j < assignments.size(); j++)
	{
		Eigen::Vector4d latest_x;
		Eigen::Vector4d measured_x;
		Eigen::Vector4d updated_filterd_x;
		latest_x << tracked_radar_objects_[assignments.at(j).first]->latest_anchor_point.head(2),
			tracked_radar_objects_[assignments.at(j).first]->latest_velocity.head(2);
		measured_x << new_radar_objects[assignments.at(j).second]->measured_anchor_point.head(2),
			new_radar_objects[assignments.at(j).second]->measured_velocity.head(2);

		kalman_filter->DoFilter(&latest_x, &measured_x, EmSensorType::SENSOR_RADAR, &updated_filterd_x);
		//printf("latest:x=%lf,y=%lf,vx=%lf,vy=%lf  mesaured:x=%lf,y=%lf,vx=%lf,vy=%lf updated:x=%lf,y=%lf,vx=%lf,vy=%lf\n",
		//	latest_x(0),latest_x(1),latest_x(2),latest_x(3),
		//	measured_x(0), measured_x(1), measured_x(2), measured_x(3),
		//	updated_filterd_x(0), updated_filterd_x(1), updated_filterd_x(2), updated_filterd_x(3));

		tracked_radar_objects_[assignments.at(j).first]->latest_anchor_point
			<< updated_filterd_x(0), updated_filterd_x(1), 0.0;
		tracked_radar_objects_[assignments.at(j).first]->latest_velocity
			<< updated_filterd_x(2), updated_filterd_x(3), 0.0;

		//tracked_radar_objects_[assignments.at(j).first]->latest_anchor_point
		//	<< measured_x(0), measured_x(1), 0.0;
		//tracked_radar_objects_[assignments.at(j).first]->latest_velocity
		//	<< measured_x(2), measured_x(3), 0.0;
		//printf("%d vs %d \n ", assignments.at(j).first, assignments.at(j).second);
		tracked_radar_objects_[assignments.at(j).first]->start_time= new_radar_objects[assignments.at(j).second]->time_stamp;
		tracked_radar_objects_[assignments.at(j).first]->latest_time = tracked_radar_objects_[assignments.at(j).first]->start_time;
	}
	//6. 未匹配新增对象
	for (size_t j = 0; j < unassigned_cols.size(); j++)
	{
		TrackedObjectPtr new_object = std::make_shared<CTrackObject>();
		memcpy(&(new_object->target_status), &(new_radar_objects[j]->target_status), sizeof(StrTargetStatus));
		new_object->start_time = new_radar_objects[j]->time_stamp;
		new_object->latest_time = new_radar_objects[j]->time_stamp;
		//设置当前车辆位置
		new_object->latest_vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
		new_object->latest_vehicle_angle = latest_vehicle_location.yaw - 90;
		//如果新增对象可能和跟踪对象存在范围交集，则更新交集对象的位置和尺寸，如果没有交集则新增
		UpdateSizeOrAddNewObject(new_object);

	}
	//7. 删除过期对象
	std::vector<TrackedObjectPtr>::iterator iter;
	for (iter = tracked_radar_objects_.begin(); iter != tracked_radar_objects_.end();)
	{
		TrackedObjectPtr track_object = *iter;
		int elapsed = track_object->latest_time - track_object->start_time;
		if ( elapsed > INVISBLE_LIMIT)
			iter = tracked_radar_objects_.erase(iter);
		else
			iter++;
	}
	int track_size = tracked_radar_objects_.size();
	printf("当前跟踪对象%d\n", track_size);
	return true;
}
bool CTracker::TrackLidar(UpdateTargetData* update_sersor_data)
{
	//这里开始障碍物跟踪
	//1. 初始化损失矩阵
	LossMat<float> loss_lidr_mat;
	loss_lidr_mat.Resize(tracked_lidar_objects_.size(), update_sersor_data->lidar_count);
	row_tag_.assign(loss_lidr_mat.height(), 0);
	col_tag_.assign(loss_lidr_mat.width(), 0);
	std::vector<UpdateObjectPtr> new_lidar_objects;
	CollectSensorData(&new_lidar_objects, update_sersor_data, EmSensorType::SENSOR_LIDAR);

	//2.计算lidar损失矩阵
	for (int i = 0; i < tracked_lidar_objects_.size(); i++)
	{
		for (int j = 0; j < new_lidar_objects.size(); j++)
		{
			UpdateObjectPtr new_object = new_lidar_objects[j];
			//设置当前车辆位置
			new_object->vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
			new_object->vehicle_angle = latest_vehicle_location.yaw - 90;

			tracked_lidar_objects_[i]->PredictState(update_sersor_data->target_data.ullTimestamp);

			loss_lidr_mat(i, j) = ComputeDistance(tracked_lidar_objects_[i],
				new_object,
				Enum_DISTANCE_LOSS_TYPE::Center);// | Enum_DISTANCE_LOSS_TYPE::Corners);
			printf("%d和%d距离=%lf\n", i, j, loss_lidr_mat(i, j));
		}

	}

	//3. 执行GNN匹配
	std::vector<std::pair<size_t, size_t>> assignments;
	std::vector<size_t> unassigned_rows;
	std::vector<size_t> unassigned_cols;
	Match(&loss_lidr_mat, &assignments, &unassigned_rows, &unassigned_cols);

	//4.给匹配对象赋分
	for (size_t i = 0; i < assignments.size(); ++i) {
		new_lidar_objects[assignments.at(i).second]->association_score =
			loss_lidr_mat(assignments.at(i).first,
			assignments.at(i).second) / max_match_distance_;
	}
	//5.执行卡尔曼滤波，基于车体坐标系滤波
	for (size_t j = 0; j < assignments.size(); j++)
	{
		Eigen::Vector4d latest_x;
		Eigen::Vector4d measured_x;
		Eigen::Vector4d updated_filterd_x;
		latest_x << tracked_lidar_objects_[assignments.at(j).first]->latest_anchor_point.head(2),
			tracked_lidar_objects_[assignments.at(j).first]->latest_velocity.head(2);
		measured_x << new_lidar_objects[assignments.at(j).second]->measured_anchor_point.head(2),
			new_lidar_objects[assignments.at(j).second]->measured_velocity.head(2);
		float delta_time = update_sersor_data->target_data.ullTimestamp - tracked_lidar_objects_[assignments.at(j).first]->latest_time;
		kalman_filter->DoFilter(&latest_x, &measured_x, EmSensorType::SENSOR_LIDAR, &updated_filterd_x , delta_time/1000.0);

		printf("latest:x=%lf,y=%lf,vx=%lf,vy=%lf\n", latest_x(0), latest_x(1), latest_x(2), latest_x(3));
		printf("predicted:x=%lf,y=%lf,vx=%lf,vy=%lf\n",
			tracked_lidar_objects_[assignments.at(j).first]->predict_.state(0),
			tracked_lidar_objects_[assignments.at(j).first]->predict_.state(1),
			tracked_lidar_objects_[assignments.at(j).first]->predict_.state(3),
			tracked_lidar_objects_[assignments.at(j).first]->predict_.state(4));
		printf("mesaured:x=%lf,y=%lf,vx=%lf,vy=%lf\n", measured_x(0), measured_x(1), measured_x(2), measured_x(3));
		printf("updated:x=%lf,y=%lf,vx=%lf,vy=%lf\n", updated_filterd_x(0), updated_filterd_x(1), updated_filterd_x(2), updated_filterd_x(3));
		printf("%d vs %d \n ", assignments.at(j).first, assignments.at(j).second);

		tracked_lidar_objects_[assignments.at(j).first]->latest_anchor_point 
			<< updated_filterd_x(0), updated_filterd_x(1), 0.0;
		tracked_lidar_objects_[assignments.at(j).first]->latest_velocity
			<<  updated_filterd_x(2), updated_filterd_x(3), 0.0;

		tracked_lidar_objects_[assignments.at(j).first]->start_time = new_lidar_objects[assignments.at(j).second]->time_stamp;
		tracked_lidar_objects_[assignments.at(j).first]->latest_time = tracked_lidar_objects_[assignments.at(j).first]->start_time;

	}
	//6. 匹配新增对象
	for (size_t j = 0; j < unassigned_cols.size(); j++)
	{
		TrackedObjectPtr new_object = std::make_shared<CTrackObject>();
		memcpy(&(new_object->target_status), &(new_lidar_objects[j]->target_status), sizeof(StrTargetStatus));
		new_object->start_time = new_lidar_objects[j]->time_stamp;
		new_object->latest_time = new_lidar_objects[j]->time_stamp;

		//设置当前车辆位置
		new_object->latest_vehicle_pos << latest_vehicle_location.fX, latest_vehicle_location.fY, latest_vehicle_location.fZ;
		new_object->latest_vehicle_angle = latest_vehicle_location.yaw - 90;

		PushTrackObject(new_object);

	}
	//7. 删除过期对象
	std::vector<TrackedObjectPtr>::iterator iter;
	for (iter = tracked_lidar_objects_.begin(); iter != tracked_lidar_objects_.end();)
	{
		TrackedObjectPtr track_object = *iter;
		//刷新track object的跟踪时间
		track_object->latest_time = update_sersor_data->target_data.ullTimestamp;
		if (track_object->latest_time - track_object->start_time > INVISBLE_LIMIT)
			iter = tracked_lidar_objects_.erase(iter);
		else
			iter++;
	}


	int track_size = tracked_lidar_objects_.size();
	printf("当前跟踪激光雷达对象%d\n", track_size);
	return true;
}
void CTracker::PrepareData(Json::Value RecvGPS, Json::Value UpdatedSense , UpdateTargetData&  update_sersor_data)
{
	//转换收到的JSON格式
	iJsonToCarStatus(RecvGPS, &latest_vehicle_location);
	iJsonToTargetsStatus(UpdatedSense, &update_sersor_data);
	//目前感知数据并没有带时间戳，借用GPS当前时间，周内秒
	update_sersor_data.target_data.ullTimestamp = latest_vehicle_location.ullTimeStamp;
}
bool CTracker::Track(UpdateTargetData&  update_sersor_data)
{
	//如果是第一次执行跟踪，则将待跟踪障碍物全部放入跟踪列表
	if (!b_initialized)
	{
		InitTrackObjects(&update_sersor_data);
		b_initialized = true;
		return true;
	}
	TrackLidar(&update_sersor_data);
	TrackRadar(&update_sersor_data);
	return true;
}
void CTracker::Match(LossMat<float>* cost_matrix,
	std::vector<std::pair<size_t, size_t>>* assignments,
	std::vector<size_t>* unassigned_rows,
	std::vector<size_t>* unassigned_cols)
{
	float max_dist = 4.0f;

	assignments->clear();
	unassigned_rows->clear();
	unassigned_cols->clear();
	row_tag_.clear();
	col_tag_.clear();

	int num_rows = static_cast<int>(cost_matrix->height());
	int num_cols = static_cast<int>(cost_matrix->width());
	row_tag_.assign(num_rows, 0);
	col_tag_.assign(num_cols, 0);

	//找出矩阵里所有损失值小于max_dist的元素，记录行、列以及损失值
	std::vector<MatchCost> match_costs;
	for (int r = 0; r < num_rows; r++) {
		for (int c = 0; c < num_cols; c++) {
			if ((*cost_matrix)(r, c) < max_dist) {
				MatchCost item(r, c, (*cost_matrix)(r, c));
				match_costs.push_back(item);
			}
		}
	}
	// 按照升序排列
	std::sort(match_costs.begin(), match_costs.end());

	// 执行GNN匹配
	for (size_t i = 0; i < match_costs.size(); ++i) {
		size_t rid = match_costs[i].RowIdx();
		size_t cid = match_costs[i].ColIdx();
		if (row_tag_[rid] == 0 && col_tag_[cid] == 0) {
			row_tag_[rid] = 1;
			col_tag_[cid] = 1;
			assignments->push_back(std::make_pair(rid, cid));
		}
	}

	for (int i = 0; i < num_rows; i++) {
		if (row_tag_[i] == 0) {
			unassigned_rows->push_back(i);
		}
	}

	for (int i = 0; i < num_cols; i++) {
		if (col_tag_[i] == 0) {
			unassigned_cols->push_back(i);
		}
	}
}
float  CTracker::ComputeDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object,  int loss_type)
{
	float loss = 0.0;
	if ((loss_type&Enum_DISTANCE_LOSS_TYPE::Center) == Enum_DISTANCE_LOSS_TYPE::Center)
	{
		loss += ComputeLocationDistance(tracked, new_object);
	}
	if ((loss_type&Enum_DISTANCE_LOSS_TYPE::Corners) == Enum_DISTANCE_LOSS_TYPE::Corners)
	{
		loss += ComputeCornersDistance(tracked, new_object);
	}
	if ((loss_type&Enum_DISTANCE_LOSS_TYPE::Iou) == Enum_DISTANCE_LOSS_TYPE::Iou)
	{
		loss += ComputeIOUDistance(tracked, new_object);
	}
	return loss;
}
float CTracker::ComputeLocationDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object)
{
	Eigen::Vector3d measured_anchor_point = (new_object->measured_anchor_point).cast<double>();
	Eigen::Vector3d predicted_anchor_point = tracked->predict_.state.head(3).cast<double>();

	//将坐标转换到本地ENU坐标系进行损失计算，没必要！
	//CGeoUtil::Vehicle2Enu(measured_anchor_point, tracked->latest_vehicle_pos, tracked->latest_vehicle_angle, measured_anchor_point);
	//CGeoUtil::Vehicle2Enu(predicted_anchor_point, new_object->vehicle_pos, new_object->vehicle_angle, predicted_anchor_point);

	Eigen::Vector3d measure_predict_diff =	measured_anchor_point - predicted_anchor_point;

	float location_dist = static_cast<double>(sqrt(
		(measure_predict_diff.head(2).cwiseProduct(measure_predict_diff.head(2)))
		.sum()));
	if (location_dist < 1e-05)
		return location_dist;

	//如果距离超过阈值，并且速度非空，则进行调整，惩罚非运动方向的距离
	//对于lidar数据，是从第二帧开始匈牙利匹配后估算的速度，是相对车辆的速度
	Eigen::Vector2d ref_dir = tracked->latest_velocity.head(2);
	double norm = ref_dir.norm();
	if (norm > 1e-5)
	{
		ref_dir /= norm;
		Eigen::Vector2d ref_o_dir = Eigen::Vector2d(ref_dir[1], -ref_dir[0]);
		double dx = ref_dir(0) * measure_predict_diff(0) +
			ref_dir(1) * measure_predict_diff(1);
		double dy = ref_o_dir(0) * measure_predict_diff(0) +
			ref_o_dir(1) * measure_predict_diff(1);
		location_dist = static_cast<float>(sqrt(dx * dx * 0.5 + dy * dy * 2));
	}
	
	return location_dist;

}
float CTracker::ComputeCornersDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object)
{
	float corners_distance = 0.0;
	for (int i = 0; i < 4; i++)
	{
		Eigen::Vector3f measured_point =
			(new_object->measured_corners[i]).cast<float>();
		Eigen::Vector3f predicted_point = tracked->predict_.state.head(6+(i+1)*3).tail(3);
		Eigen::Vector3f measure_predict_diff = measured_point - predicted_point;
		corners_distance += static_cast<float>(sqrt(
			(measure_predict_diff.head(2).cwiseProduct(measure_predict_diff.head(2)))
			.sum()));
	}
	return corners_distance / 4.0;
}
float CTracker::ComputeIOUDistance(TrackedObjectPtr tracked, UpdateObjectPtr new_object)
{

	float iou_distance = 0.0;
	DeepBlue::Extent2D tracked_extent = tracked->GetObjectEnvlope();
	DeepBlue::Extent2D new_object_extent = new_object->GetObjectEnvlope();
	DeepBlue::Extent2D inside_box, outside_box;
	//计算交集
	inside_box.min_x = std::max(tracked_extent.min_x, new_object_extent.min_x);
	inside_box.max_x = std::min(tracked_extent.max_x, new_object_extent.max_x);
	inside_box.min_y = std::max(tracked_extent.min_y, new_object_extent.min_y);
	inside_box.max_y = std::min(tracked_extent.max_y, new_object_extent.max_y);

	//计算并集
	outside_box.min_x = std::min(tracked_extent.min_x, new_object_extent.min_x);
	outside_box.max_x = std::max(tracked_extent.max_x, new_object_extent.max_x);
	outside_box.min_y = std::min(tracked_extent.min_y, new_object_extent.min_y);
	outside_box.max_y = std::max(tracked_extent.max_y, new_object_extent.max_y);

	//如果有交集部分
	if (inside_box.valid()&&outside_box.valid())
	{
		iou_distance = inside_box.area() / outside_box.area();
	}
	return (1 - iou_distance)* background_object_match_threshold_;
}
void CTracker::CollectSensorData(std::vector<UpdateObjectPtr>* new_objects , UpdateTargetData* sensor_data, EmSensorType sensor_type)
{
	for (int i = 0; i < sensor_data->total_count; i++)
	{
		if (sensor_data->target_data.strTargetStatus[i].emSensorType == sensor_type)
		{
			UpdateObjectPtr update_object = WrapSensorData(&(sensor_data->target_data.strTargetStatus[i]));
			update_object->time_stamp = sensor_data->target_data.ullTimestamp;
			new_objects->push_back(update_object);
		}
	}
}
UpdateObjectPtr CTracker::WrapSensorData(StrTargetStatus* update_status)
{
	UpdateObjectPtr sensor_data = std::make_shared<CSensorData>();
	memcpy(&(sensor_data->target_status), update_status, sizeof(StrTargetStatus));
	sensor_data->measured_anchor_point(0) = update_status->fX;
	sensor_data->measured_anchor_point(1) = update_status->fY;
	sensor_data->measured_anchor_point(2) = 0.0;
	sensor_data->measured_velocity(0) = update_status->fRate*cos(update_status->fAngle);
	sensor_data->measured_velocity(1) = update_status->fRate*cos(update_status->fAngle);
	sensor_data->measured_velocity(2) = 0.0;


	for (int i = 0; i < 4; i++)
	{
		sensor_data->measured_corners[i](0) = update_status->strTargetBoard.strTargetPoint[i].fX;
		sensor_data->measured_corners[i](1) = update_status->strTargetBoard.strTargetPoint[i].fY;
		sensor_data->measured_corners[i](2) = 0.0;
	}
	return sensor_data;
}

void CTracker::PackFusionObjects(Json::Value& funsion_sense)
{
	//将RADAR目标中和激光雷达目标重合的剔除
	std::vector<TrackedObjectPtr>::iterator iter1;
	std::vector<TrackedObjectPtr>::iterator iter2;
	bool del_flag;
	for (iter1 = tracked_radar_objects_.begin(); iter1 != tracked_radar_objects_.end();)
	{
		del_flag = false;
		TrackedObjectPtr track_radar_object = *iter1;

		for (iter2 = tracked_lidar_objects_.begin(); iter2 != tracked_lidar_objects_.end();iter2++)
		{
			TrackedObjectPtr track_lidar_object = *iter2;
			if (DeepBlue::CVectorUtil::Extent2dIntersected(track_radar_object->GetObjectEnvlope(), track_lidar_object->GetObjectEnvlope()))
			{
				iter1 = tracked_radar_objects_.erase(iter1);
				del_flag = true;
				break;
			}
		}
		if (!del_flag)
			iter1++;
	}

	//打包成JSON格式
	iTargetsStatusToJson(funsion_sense);

}
bool CTracker::iTargetsStatusToJson(Json::Value& funsion_sense)
{
	Json::Value lidar_objects;
	float dw = 0.0;
	float dl = 0.0;
	for (int i = 0; i < tracked_lidar_objects_.size(); i++)
	{
		Json::Value object;
		object["ID"] = tracked_lidar_objects_[i]->target_status.iTargetId;
		object["X"] = tracked_lidar_objects_[i]->target_status.fX;
		object["Y"] = tracked_lidar_objects_[i]->target_status.fY;
		object["SX"] = tracked_lidar_objects_[i]->target_status.fRate;
		object["A"] = tracked_lidar_objects_[i]->target_status.fAngle;
		object["W"] = tracked_lidar_objects_[i]->target_status.fWidth;
		object["L"] = tracked_lidar_objects_[i]->target_status.fLength;
		dw = tracked_lidar_objects_[i]->target_status.fWidth / 2.0;
		dl = tracked_lidar_objects_[i]->target_status.fLength / 2.0;

		object["P1_X"] = tracked_lidar_objects_[i]->latest_anchor_point(0) + dl;
		object["P1_Y"] = tracked_lidar_objects_[i]->latest_anchor_point(1) - dw;
		object["P2_X"] = tracked_lidar_objects_[i]->latest_anchor_point(0) - dl;
		object["P2_Y"] = tracked_lidar_objects_[i]->latest_anchor_point(1) - dw;
		object["P3_X"] = tracked_lidar_objects_[i]->latest_anchor_point(0) - dl;
		object["P3_Y"] = tracked_lidar_objects_[i]->latest_anchor_point(1) + dw;
		object["P4_X"] = tracked_lidar_objects_[i]->latest_anchor_point(0) + dl;
		object["P4_Y"] = tracked_lidar_objects_[i]->latest_anchor_point(1) + dw;

		//object["P1_X"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[0].fX;
		//object["P1_Y"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[0].fY;
		//object["P2_X"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[1].fX;
		//object["P2_Y"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[1].fY;
		//object["P3_X"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[2].fX;
		//object["P3_Y"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[2].fY;
		//object["P4_X"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[3].fX;
		//object["P4_Y"] = tracked_lidar_objects_[i]->target_status.strTargetBoard.strTargetPoint[3].fY;
		lidar_objects.append(object);
	}

	Json::Value radar_objects;
	for (int i = 0; i < tracked_radar_objects_.size(); i++)
	{
		Json::Value object;
		object["ID"] = tracked_radar_objects_[i]->target_status.iTargetId;
		object["X"] = tracked_radar_objects_[i]->target_status.fX;
		object["Y"] = tracked_radar_objects_[i]->target_status.fY;
		object["SX"] = tracked_radar_objects_[i]->target_status.fRate;
		object["A"] = tracked_radar_objects_[i]->target_status.fAngle;
		object["W"] = tracked_radar_objects_[i]->target_status.fWidth;
		object["L"] = tracked_radar_objects_[i]->target_status.fLength;
		radar_objects.append(object);
	}
    funsion_sense["SENSE"]["NUM_LIDAR_OBJECTS"] = static_cast<int>(tracked_lidar_objects_.size());
	funsion_sense["SENSE"]["LIDAR_OBJECTS"] = lidar_objects;

    funsion_sense["SENSE"]["NUM_RADAR_OBJECTS"] = static_cast<int>(tracked_radar_objects_.size());
	funsion_sense["SENSE"]["RADAR_OBJECTS"] = radar_objects;

	return true;
}
int CTracker::iJsonToCarStatus(Json::Value RecvGPS, StrLocationFusion *pStrCarStatus)
{


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
	pStrCarStatus->ullTimeStamp = RecvGPS["GPS_MS"].asInt64();
	
	//计算车辆当前ENU位置和方位角，方位角缺省为90度，所以需要旋转的角度=angle-90
	Point v_enu_point;
	MapPoint wgs_point;
	memset(&wgs_point, 0, sizeof(MapPoint));
	wgs_point.Lat = pStrCarStatus->dLatitude;
	wgs_point.Lon = pStrCarStatus->dLongitude;
	wgs_point.Alt = 0.0;
	CGeoUtil::WGS842Enu(OriginPoint, wgs_point, &v_enu_point);
	///////////////////////////////////////////////////////////////////////////////

	pStrCarStatus->fX = v_enu_point.X;
	pStrCarStatus->fY = v_enu_point.Y;
	pStrCarStatus->fZ = v_enu_point.Z;

	
	string NAV_Status = RecvGPS["NAV_STATE"].asString();
	string GPS_Status = RecvGPS["GPS_STATE"].asString();

    int iNavStatus = 0;
    if (!NAV_Status.empty()) {
        iNavStatus = std::stoi(NAV_Status);
    }
    int iGpsStatus = 0;
    if (!GPS_Status.empty()) {
        iGpsStatus = std::stoi(GPS_Status);
    }

	if ((iNavStatus >= 1) && (iGpsStatus >= 1))
	{
		pStrCarStatus->cSystemStatus = 1;
	}
	else
	{
		pStrCarStatus->cSystemStatus = 0;
	}

	return 0;
}

int CTracker::iJsonToTargetsStatus(Json::Value RecvSense, UpdateTargetData *pUpdateSersorData)
{
	//计算车辆当前ENU位置和方位角，方位角缺省为90度，所以需要旋转的角度=angle-90
	Point v_enu_point;
	MapPoint wgs_point;
	memset(&wgs_point, 0, sizeof(MapPoint));
	wgs_point.Lat = latest_vehicle_location.dLatitude;
	wgs_point.Lon = latest_vehicle_location.dLongitude;
	wgs_point.Alt = 0.0;
	CGeoUtil::WGS842Enu(OriginPoint, wgs_point, &v_enu_point);
	///////////////////////////////////////////////////////////////////////////////

	Point tmp_point;

	StrTargetFusion *pTargetsFusion = &(pUpdateSersorData->target_data);
	int i = 0;

	int iRadarNum = pUpdateSersorData->radar_count = RecvSense["NUM_RADAR_OBJECTS"].asInt();
	int iCameraNum = pUpdateSersorData->camera_count = 0;// RecvSense["NUM_CAMERA_OBJECTS"].asInt();
	int iLidarNum = pUpdateSersorData->lidar_count = RecvSense["NUM_LIDAR_OBJECTS"].asInt();
	pUpdateSersorData->total_count = iRadarNum + iCameraNum + iLidarNum;

	if (iRadarNum + iCameraNum + iLidarNum == 0)
	{
		pTargetsFusion->iTargetNum = 0;
		//return 0;
	}
	//障碍物数量超出MAX_TARGET_NUM
	if (iRadarNum + iCameraNum + iLidarNum >= MAX_TARGET_NUM)
	{
		cout << "too many obstacles ,stop fusing \n" << endl;
		return -1;
	}

	pTargetsFusion->iTargetNum = iRadarNum + iCameraNum + iLidarNum;

	Json::Value CameraObjects = RecvSense["CAMERA_OBJECTS"];
	Point o_v_pt, o_enu_pt;
	float vx = 0.0;
	float vy = 0.0;
	float px = 0.0;
	float py = 0.0;
	float enu_px = 0.0;
	float enu_py = 0.0;
	float enu_vx = 0.0;
	float enu_vy = 0.0;
	float frate = 0.0;
	float enu_frate = 0.0;
	for (i = 0; i < iCameraNum; i++)
	{
		Json::Value Object = CameraObjects[i];
		pTargetsFusion->strTargetStatus[i].emSensorType = SENSOR_CAMERA;

		o_v_pt.X = Object["X"].asFloat();
		o_v_pt.Y = Object["Y"].asFloat();
		CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw-90, o_enu_pt);

		pTargetsFusion->strTargetStatus[i].fX = o_enu_pt.X;
		pTargetsFusion->strTargetStatus[i].fY = o_enu_pt.Y;
		//pTargetsFusion->strTargetStatus[i].fX = Object["X"].asFloat();
		//pTargetsFusion->strTargetStatus[i].fY = Object["Y"].asFloat();
		pTargetsFusion->strTargetStatus[i].iTargetId = i;
		//pTargetsFusion->strTargetStatus[i].fRate = Object["RR"].asFloat(); //根本就没有这项数据
		
		//将雷达测量数据换算到ENU坐标系
		vx = Object["V_REL_LONG"].asFloat();
		vy = Object["V_REL_LAT"].asFloat();
		px = Object["X"].asFloat();
		py = Object["Y"].asFloat();

		if ((px + py) < 1e-08)
			pTargetsFusion->strTargetStatus[i].fRate = 0.0;
		else
			pTargetsFusion->strTargetStatus[i].fRate = (vx*px + vy*py) / sqrt(px*px + py*py);
		//////////////////////////////////////////////////////////////////////////////////////////////

		pTargetsFusion->strTargetStatus[i].fAngle = Object["A"].asFloat();
		pTargetsFusion->strTargetStatus[i].fAngleRate = Object["LR"].asFloat();
	}

	Json::Value RadarObjects = RecvSense["RADAR_OBJECTS"];

	for (i = 0; i < iRadarNum; i++)
	{
		Json::Value Object = RadarObjects[i];
		pTargetsFusion->strTargetStatus[i + iCameraNum].emSensorType = SENSOR_RADAR;
		o_v_pt.X = Object["X"].asFloat();
		o_v_pt.Y = Object["Y"].asFloat();
		CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);

		//pTargetsFusion->strTargetStatus[i + iCameraNum].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum].fY = o_enu_pt.Y;
		//车体坐标系下进行卡尔曼滤波，跟踪损失计算转换到ENU本地坐标系
		pTargetsFusion->strTargetStatus[i + iCameraNum].fX = Object["X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum].fY = Object["Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum].iTargetId = iRadarNum + i;
		
		//pTargetsFusion->strTargetStatus[i + iCameraNum].fRate = Object["RR"].asFloat();//根本就没这项数据

		//换算为雷达的测量数据
		vx = Object["V_REL_LONG"].asFloat();
		vy = Object["V_REL_LAT"].asFloat();
 
		px = Object["X"].asFloat();
		py = Object["Y"].asFloat();
		enu_px = o_enu_pt.X;
		enu_py = o_enu_pt.Y;

		if ((px + py) < 1e-08)
			pTargetsFusion->strTargetStatus[i].fRate = 0.0;
		else
			pTargetsFusion->strTargetStatus[i].fRate = (vx*px + vy*py) / sqrt(px*px + py*py);
		//////////////////////////////////////////////////////////////////////////////////////////////

		pTargetsFusion->strTargetStatus[i + iCameraNum].fAngle = Object["A"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum].fAngleRate = Object["LR"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum].fWidth = 1.0;// Object["W"].asFloat(); //default 
		pTargetsFusion->strTargetStatus[i + iCameraNum].fLength = 1.0;// Object["L"].asFloat();
	}

	Json::Value LidarObjects = RecvSense["LIDAR_OBJECTS"];
	for (i = 0; i < iLidarNum; i++)
	{
		Json::Value Object = LidarObjects[i];
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].emSensorType = SENSOR_LIDAR;

		//transform to world coordinate
		o_v_pt.X = Object["X"].asFloat();
		o_v_pt.Y = Object["Y"].asFloat();
		CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);

		//pTargetsFusion->strTargetStatus[i + iCameraNum].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum].fY = o_enu_pt.Y;
		/////////////////////////////////////////////////////////////////////////////////////////////

		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fX = Object["X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fY = Object["Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].iTargetId = iRadarNum + i;
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fRate = Object["SX"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fAngle = Object["A"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fWidth = Object["W"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].fLength = Object["L"].asFloat();

		//o_v_pt.X = Object["P1_X"].asFloat();
		//o_v_pt.Y = Object["P1_Y"].asFloat();
		//CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[0].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[0].fY = o_enu_pt.Y;

		//o_v_pt.X = Object["P2_X"].asFloat();
		//o_v_pt.Y = Object["P2_Y"].asFloat();
		//CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[1].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[1].fY = o_enu_pt.Y;

		//o_v_pt.X = Object["P3_X"].asFloat();
		//o_v_pt.Y = Object["P3_Y"].asFloat();
		//CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[2].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[2].fY = o_enu_pt.Y;

		//o_v_pt.X = Object["P4_X"].asFloat();
		//o_v_pt.Y = Object["P4_Y"].asFloat();
		//CGeoUtil::Vehicle2Enu(v_enu_point, o_v_pt, latest_vehicle_location.yaw - 90, o_enu_pt);
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[3].fX = o_enu_pt.X;
		//pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[3].fY = o_enu_pt.Y;

		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[0].fX = Object["P1_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[0].fY = Object["P1_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[1].fX = Object["P2_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[1].fY = Object["P2_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[2].fX = Object["P3_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[2].fY = Object["P3_Y"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[3].fX = Object["P4_X"].asFloat();
		pTargetsFusion->strTargetStatus[i + iCameraNum + iRadarNum].strTargetBoard.strTargetPoint[3].fY = Object["P4_Y"].asFloat();
	}
	return 0;
}

