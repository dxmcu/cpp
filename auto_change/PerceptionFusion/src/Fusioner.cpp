#include <iostream>
#include <sstream>
#include <fstream>
#include "Fusioner.h"
#include "Tracker.h"
#include "zmq.h"

#pragma warning(disable:4996)  //neglect json::fasterwriter outdate warning

CFusioner::CFusioner()
{
  //将请求预先打包成Json格式
  std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
  reqDataContext["TYPE"] = "REQ_PERCEPTION_DATA_LOCAL";
  std::ostringstream os;
  jsonWriter->write(reqDataContext, &os);
  str_request_send = os.str();
  enable_debug = false;

  tracker = new CTracker();

  g_z_context = nullptr;
  g_z_socket = nullptr;
}

CFusioner::~CFusioner()
{
  if (g_z_socket)
    zmq_close(g_z_socket);
  if (g_z_context)
    zmq_term(g_z_context);

  delete tracker;
}

void CFusioner::EnableDebug()
{
  enable_debug = true;
}
bool CFusioner::InitializedZmq()
{
  //初始化ZMQ连接
  g_z_context = zmq_init(1);
  g_z_socket = zmq_socket(g_z_context, ZMQ_REQ);

  while ((zmq_connect(g_z_socket, "tcp://127.0.0.1:7777") > 0))
  {
    printf("waiting datapool service......\n");
  }
  return true;
}
bool CFusioner::SendZmqRequest()
{
  // 将Json格式数据发送给DataPool
  int length = (int)str_request_send.length();
  int iRet = zmq_send(g_z_socket, str_request_send.c_str(), length, 0);
  if (iRet != length)
  {
    printf("Perception failed to send request，Chk network connection!\n");
    return false;
  }
  else
  {
    printf("Perception successfully sent request\n");
  }
  return true;
}
void CFusioner::InitData()
{
  //清理
  recv_keep_data_.clear();
  recv_gps_.clear();
  recv_ultrasound_.clear();
  funsion_output_.clear();
  recv_sense_.clear();
}
bool CFusioner::ReceivePerceptionData()
{
  //清空上一帧的数据
  InitData();

  //接受当前感知数据
  zmq_msg_t recv_msg;
  zmq_msg_init(&recv_msg);
  zmq_msg_recv(&recv_msg, g_z_socket, 0);
  char *ch_recv = (char*)zmq_msg_data(&recv_msg);
  std::string str_recv(ch_recv);
  zmq_msg_close(&recv_msg);
  Json::Reader reader;

  if (!reader.parse(str_recv, updated_perception_value, false))
  {
    //解析失败
    std::cout << "zmq perception json parse failed" << std::endl;
    std::cout << "receive: " << str_recv.c_str() << std::endl;
    return false;
  }
  std::string type = updated_perception_value["TYPE"].asString();

  if (type == "PERCEPTION_DATA_LOCAL")
  {
    recv_keep_data_ = updated_perception_value["DATA"];
    recv_sense_ = updated_perception_value["SENSE"];
    recv_gps_ = updated_perception_value["GPS"];
    recv_ultrasound_ = recv_sense_["ULTRASOUND_OBJECTS"];
  }
  else
  {
    std::cout << "returned type is not qualified for perception fusion！" << std::endl;
    return false;
  }
  return true;
}
bool CFusioner::DoFuse(bool fuse_flag)
{
  if(fuse_flag)
  {
    UpdateTargetData  update_sersor_data;
    //将收到的数据转换成跟踪格式，坐标从车体坐标转成本地坐标ENU
    tracker->PrepareData(recv_gps_, recv_sense_, update_sersor_data);

    //跟踪并融合
    tracker->Track(update_sersor_data);

    //打包成返回的JSON格式
    tracker->PackFusionObjects(funsion_output_);

    //用来对比展示
    funsion_output_["ORIGINAL_SENSE"] = recv_sense_;
  }
  else
  {
    funsion_output_["SENSE"] = recv_sense_;
  }
  PackExtraFusionData();
  return true;
}
void CFusioner::PackExtraFusionData()
{
  //将附加信息打包回JSON
  funsion_output_["DATA"] = recv_keep_data_;
  funsion_output_["GPS"] = recv_gps_;
  //超声波总是8个
  funsion_output_["SENSE"]["ULTRASOUND_OBJECTS"] = recv_ultrasound_;
}
bool CFusioner::SendFusedResult()
{
  Json::Value rep;
  Json::FastWriter writer;

  zmq_msg_t send_msg;
  rep["TYPE"] = "PUSH_UPDATE";
  rep["DATA"] = funsion_output_;

  std::string str_send = writer.write(rep);
  int n = str_send.length();
  zmq_msg_init_size(&send_msg, n + 1);
  std::memcpy(zmq_msg_data(&send_msg), str_send.c_str(), n + 1);
  zmq_sendmsg(g_z_socket, &send_msg, 0);
  zmq_msg_close(&send_msg);
  return true;
}
bool CFusioner::ReceiveNotifyInfo()
{
  zmq_msg_t recv_msg;
  zmq_msg_init(&recv_msg);
  zmq_msg_recv(&recv_msg, g_z_socket, 0);
  char *ch_recv = (char*)zmq_msg_data(&recv_msg);
  std::string str_recv(ch_recv);
  zmq_msg_close(&recv_msg);
  Json::Reader reader;
  Json::Value value;
  if (!reader.parse(str_recv, value, false))
  {
    return false;
  }
  else
  {
    std::string type = value["TYPE"].asString();
    // 接收数据池回执信息
    if (type == "REP_OK")
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
void CFusioner::ReadLocalData(std::string path, int dataid)
{
  std::string filepath1 = path + std::to_string(dataid) + "_sense.txt";
  std::string filepath2 = path + std::to_string(dataid) + "_gps.txt";
  std::fstream f1, f2;
  Json::CharReaderBuilder rbuilder;
  JSONCPP_STRING errs;

  f1.open(filepath1.c_str(), std::ios::in);
  bool parse_ok = Json::parseFromStream(rbuilder, f1, &recv_sense_, &errs);
  if (!parse_ok)
  {
    recv_sense_.clear();
    std::cout << "Parse json sense file error!" << std::endl;
  }
  f1.close();

  f2.open(filepath2.c_str(), std::ios::in);
  parse_ok = Json::parseFromStream(rbuilder, f2, &recv_gps_, &errs);
  if (!parse_ok)
  {
    recv_sense_.clear();
    std::cout << "Parse json gps file error!" << std::endl;
  }
  f2.close();

  //int num_radar_objects = recv_sense_["NUM_RADAR_OBJECTS"].asInt();
  //cout << num_radar_objects << endl;
}
