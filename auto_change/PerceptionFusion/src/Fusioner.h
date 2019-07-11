/*
*  author: deep blue auto drive team
*    date: 2019/05/14
*  funciton: perception fusion main class
*  Modification: 
*  top7gun -   first initialization    2019-05-14
*/

#pragma once

#include <jsoncpp/json/json.h>

class CTracker;

class CFusioner
{
public:
  CFusioner();
  ~CFusioner();
public:
  bool SendZmqRequest();
  bool InitializedZmq();
  bool ReceivePerceptionData();
  bool DoFuse(bool fuse_flag = false);
  bool SendFusedResult();
  bool ReceiveNotifyInfo();
  void EnableDebug();
  void InitData();
  void ReadLocalData(std::string path, int dataids);
  void PackExtraFusionData();
public:
  Json::Value funsion_output_;
protected:
  bool enable_debug;
  Json::Value updated_perception_value;
  Json::Value reqDataContext;

  Json::Value recv_gps_;
  Json::Value recv_keep_data_;
  Json::Value recv_ultrasound_;
  Json::Value recv_sense_;

  Json::StreamWriterBuilder writerBuilder;
  std::string str_request_send;
  void* g_z_socket;
  void* g_z_context;
  CTracker* tracker;
};

