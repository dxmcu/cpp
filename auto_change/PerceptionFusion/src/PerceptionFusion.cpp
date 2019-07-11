// PerceptionFusion.cpp : Defines the entry point for the console application.
//

#include <unistd.h>
#include <iostream> 
#include "Fusioner.h"

int main(int argc, char* argv[])
{
  std::cout << "Enter the Percetion Fusion Module" << std::endl;

  bool local_logs_debug = true;
  int send_status = 0;
  CFusioner fusioner;
  if (!fusioner.InitializedZmq()) {
    std::cout << "zmq初始化失败，退出!" << std::endl;
    return 0;
  }
  //如果是本地日志调试，直接执行融合
  if (local_logs_debug)
    send_status = 1;

  int logs_count = 0;
  std::string log_path = "/home/lz/work/doc/sweeper/PerceptionJsonData/";
  while (true) {
    // 向DataPool请求待融合数据
    if(send_status == 0 && !local_logs_debug) {
      if (fusioner.SendZmqRequest())
        send_status = 1;
    }
    // 接受DATAPOOL感知数据，并进行融合处理
    if(send_status == 1) {
      if (!local_logs_debug) {
        if (!fusioner.ReceivePerceptionData()) continue;
      }
      else {
        //读取本地LOGS
        fusioner.ReadLocalData(log_path, logs_count++);
        std::cout << logs_count << std::endl;
      }
      if (local_logs_debug) {
        bool do_fuse = true;
        if (!fusioner.DoFuse(do_fuse))                {
          std::cout << "critical unrecoverable erros , exit！" << std::endl;
          return 0;
        };
        usleep(100 * 1000);
      }


      send_status = 2;
    }
    // 向DATAPOOL发送融合结果
    if (send_status == 2)        {
      if (!fusioner.SendFusedResult())            {
        std::cout << "critical unrecoverable erros , exit！" << std::endl;
        return 0;
      }
      send_status = 3;
    }
    // 接受DATAPOOL返回的接受回执
    if (send_status == 3)        {
      int error_count = 0;
      while (!fusioner.ReceiveNotifyInfo())            {
        error_count++;
        if (error_count > 3)                {
          std::cout << "receive datapool receipe info timeout! exit!" << std::endl;
          return 0;
        }
      }
      //发送成功，启动新一轮融合处理
      if (!local_logs_debug)
        send_status = 0;
      else
        send_status = 1;
    }
  }
  return 0;
}

