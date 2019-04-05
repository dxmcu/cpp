#include <string>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <jsoncpp/json/json.h>
#include "chassis_protocol.h"

using boost::asio::ip::tcp;
using boost::asio::ip::address;
using namespace chassis;

enum CmdIndex
{
    CmdHeartbeat,

    CmdStatusInfo,
    CmdStatusLoc,
    CmdStatusSpeed,
    CmdStatusBlock,
    CmdStatusBattery,
    CmdStatusTask,
    CmdStatusGetAllMaps,
    CmdStatusGetLog,
    CmdStatusGetRunningInfo,
    CmdStatusGetAllTasks,
    CmdStatusGetTaskPath,
    CmdStatusGetScan,
    CmdStatusGetParams,
    CmdStatusGetRobotStatus,
    CmdStatusGetObstacle,
    CmdStatusGetPushStatus,

    CmdControlMotion,
    CmdControlStartSlam,
    CmdControlEndSlam,
    CmdControlStartTask,
    CmdControlPauseTask,
    CmdControlResumeTask,
    CmdControlCancelTask,
    CmdControlSendGoal,
    CmdControlReloc,
    CmdControlDeleteTask,
    CmdControlUploadTask,
    CmdControlDownloadTask,
    CmdControlIoControl,
    CmdControlAutoCharge,
    CmdControlRecordTask,

    CmdConfigUploadMap,
    CmdConfigDownloadMap,
    CmdConfigDeleteMap,
    CmdConfigConfigWifi,
    CmdConfigChangeNaviMap,
    CmdConfigSetDefaultTask,
    CmdConfigRestartRobot,
    CmdConfigSetParams,
    CmdConfigRenameMap,

    CmdAllCount
};

static constexpr int CMD_REQ[] = {
    999,
    1000, 1001, 1002, 1003, 1004, 1006, 1007, 1008, 1009, 1010, 1011, 1013, 1014, 1015, 1016, 1017,
    2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014,
    3000, 3001, 3002, 3004, 3005, 3006, 3007, 3008, 3009
};
static constexpr int CMD_RES[] = {
    10999,
    11000, 11001, 11002, 11003, 11004, 11006, 11007, 11008, 11009, 11010, 11011, 11013, 11014, 11015, 11016, 11017,
    12000, 12001, 12002, 12003, 12004, 12005, 12006, 12007, 12008, 12009, 12010, 12011, 12012, 12013, 12014,
    13000, 13001, 13002, 13004, 13005, 13006, 13007, 13008, 13009
};

class session : public boost::enable_shared_from_this<session> {
public:
    session(boost::asio::io_service &io_service): socket_(io_service)
    {
        msgsend_.reset(new ProtocolMsg);
        msgrecv_.reset(new ProtocolMsg);
    }

    void start() {
        std::cout << "Start " << std::endl;
        static tcp::no_delay option(true);
        socket_.set_option(option);

        start_read();
    }
    void start_read() {
        msgrecv_->HtoN();
        boost::asio::async_read(socket_,
                                boost::asio::buffer(&(msgrecv_->m_proHeader), sizeof(msgrecv_->m_proHeader)),
                                boost::bind(&session::handle_read, shared_from_this(), boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred)
                                );
    }
    
    void handle_read(boost::system::error_code errCode, std::size_t size) {
        if(!errCode)
        {
            msgrecv_->NtoH();

            if (msgrecv_->m_proHeader.m_length > 0) {
                recv_body();
            }
            else
                send_msg();
        }
        else
            std::cout << " read error:  " << errCode.message() << std::endl;
    }

    tcp::socket &socket() {
        return socket_;
    }

private:
    void recv_body() {
        uint32_t len = msgrecv_->m_proHeader.m_length;
        buffer_.resize(len);
        boost::asio::async_read(socket_,
                                boost::asio::buffer(buffer_.data(), buffer_.size()),
                                boost::bind(&session::read_body, shared_from_this(), boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred)
                                );
    }
    void read_body(boost::system::error_code errCode, std::size_t size) {
        if(!errCode) {
            BOOST_ASSERT(buffer_.size() == size);
            std::cout << "Recive Body len: " << msgrecv_->m_proHeader.m_length << std::endl;
            msgrecv_->m_strJsonBody.assign(buffer_.data(), buffer_.size());
            send_msg();
        }
        else {
            std::cout << "read body error!" << std::endl;
        }
    }
    void send_msg() {
        SetResponseMsg();
        std::cout << "Send data: " << "  Body len: " << msgsend_->m_proHeader.m_length << \
                     "  Type: " << msgsend_->m_proHeader.m_type << " number: " << msgsend_->m_proHeader.m_number << \
                     "  sync: " << (int)msgsend_->m_proHeader.m_sync << std::endl;
        uint32_t uLength = msgsend_->m_proHeader.m_length;
        msgsend_->HtoN();

        std::string data((const char*)&msgsend_->m_proHeader, sizeof(msgsend_->m_proHeader));

        if(uLength > 0)
        {
            std::cout << "string length: " << msgsend_->m_strJsonBody.size() << " header size: " << uLength << std::endl;
            BOOST_ASSERT(msgsend_->m_strJsonBody.size() == uLength);
            //vecBufs.push_back(boost::asio::const_buffer(msgsend_->m_strJsonBody.c_str(), msgsend_->m_strJsonBody.size()));
            data.append(msgsend_->m_strJsonBody);
        }

        boost::asio::async_write(socket_, boost::asio::buffer(data),
                                 boost::bind(&session::handle_write, shared_from_this(), boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
    }
    void handle_write(boost::system::error_code errCode, std::size_t size) {
        if(!errCode)
        {
            std::cout << "send data len: " << size << std::endl << std::endl;;
            start_read();
        }
        else
        {
        }
        msgsend_->NtoH();
    }

private:
    tcp::socket socket_;

    std::unique_ptr<ProtocolMsg> msgrecv_;
    std::unique_ptr<ProtocolMsg> msgsend_;

    std::vector<char> buffer_;
    static double  xpos_;
    static double  ypos_;
    static double  angle_;


    void SetResponseMsg()
    {
        msgsend_->m_proHeader = msgrecv_->m_proHeader;
        msgsend_->m_proHeader.m_type = msgrecv_->m_proHeader.m_type + 10000;
        uint16_t type = msgrecv_->m_proHeader.m_type;

        switch (type) {
        case CMD_REQ[CmdHeartbeat]:
        case CMD_REQ[CmdStatusInfo]:
        case CMD_REQ[CmdStatusBlock]:
        case CMD_REQ[CmdStatusGetLog]:
        case CMD_REQ[CmdStatusGetRunningInfo]:
        case CMD_REQ[CmdStatusGetAllTasks]:
        case CMD_REQ[CmdStatusGetTaskPath]:
        case CMD_REQ[CmdStatusGetScan]:
        case CMD_REQ[CmdStatusGetParams]:
        case CMD_REQ[CmdStatusGetRobotStatus]:
        case CMD_REQ[CmdStatusGetObstacle]:
        case CMD_REQ[CmdStatusGetPushStatus]:
            msgsend_->m_proHeader.m_length = 0;
            break;

        case CMD_REQ[CmdStatusLoc]:
        {
            Json::Value root;
            root["x"] = xpos_;
            root["y"] = ypos_;
            root["angle"] = angle_;

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdStatusSpeed]:
        {
            Json::Value root;
            root["vx"] = 0.3;
            root["vy"] = 0.1;
            root["vth"] = 0.4;

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdStatusBattery]:
        {
            Json::Value root;
            root["battery_level"] = 0.65;
            root["battery_temp"] = 45;
            root["charging"] = false;
            root["voltage"] = 36;
            root["current"] = 34;

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdStatusTask]:
        {
            Json::Value root;
            root["task_id"] = 2;
            root["task_state"] = "TaskFinishedState";

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdStatusGetAllMaps]:
        {
            Json::Value root, maps;

            maps.append("map_1");
            maps.append("map_2");
            maps.append("map_3");
            root["maps"] = maps;

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdControlMotion]:
        case CMD_REQ[CmdControlStartSlam]:
        case CMD_REQ[CmdControlEndSlam]:
        case CMD_REQ[CmdControlStartTask]:
        case CMD_REQ[CmdControlPauseTask]:
        case CMD_REQ[CmdControlResumeTask]:
        case CMD_REQ[CmdControlCancelTask]:
        case CMD_REQ[CmdControlSendGoal]:
        case CMD_REQ[CmdControlReloc]:
        case CMD_REQ[CmdControlDeleteTask]:
        case CMD_REQ[CmdControlUploadTask]:
        case CMD_REQ[CmdControlDownloadTask]:
        case CMD_REQ[CmdControlIoControl]:
        case CMD_REQ[CmdControlAutoCharge]:
        case CMD_REQ[CmdControlRecordTask]:
            msgsend_->m_proHeader.m_length = 0;
            break;

        case CMD_REQ[CmdConfigUploadMap]:
        case CMD_REQ[CmdConfigConfigWifi]:
        case CMD_REQ[CmdConfigChangeNaviMap]:
        case CMD_REQ[CmdConfigSetDefaultTask]:
        case CMD_REQ[CmdConfigRestartRobot]:
        case CMD_REQ[CmdConfigSetParams]:
        case CMD_REQ[CmdConfigRenameMap]:
            msgsend_->m_proHeader.m_length = 0;
            break;

        case CMD_REQ[CmdConfigDownloadMap]:
        {
            Json::Reader reader;
            Json::Value root;
            if(!reader.parse(msgrecv_->m_strJsonBody, root)) return;
            std::string strMapName = root["map_name"].asString();

            Json::Value data;
            Json::Value header;
            header["map_name"] = strMapName;
            header["map_type"] = "2D-Map";
            Json::Value max_pos;
            max_pos["x"] = 19.7999992370605;
            max_pos["y"] = 19.7999992370605;
            header["max_pos"] = max_pos;
            Json::Value min_pos;
            max_pos["x"] = 19.7999992370605;
            max_pos["y"] = 19.7999992370605;
            header["min_pos"] = min_pos;
            header["resolution"] = 0.0500000007450581;
            header["encode"] = "data:image/png;base64,";
            data["header"] = header;

            Json::Value element, waypoints;
            Json::Value item;
            item["name"] = "p1";
            item["type"] = "nav_point";
            Json::Value pos;
            pos["x"] = 0.2;
            pos["y"] = 0.9;
            pos["angle"] = 0.1;
            item["pos"] = pos;
            waypoints.append(item);

            Json::Value item2;
            item2["name"] = "p2";
            item2["type"] = "nav_point";
            Json::Value pos2;
            pos2["x"] = 32.5;
            pos2["y"] = 23.3;
            pos2["angle"] = 0.1;
            item2["pos"] = pos2;
            waypoints.append(item2);

            Json::Value item3;
            item2["name"] = "p3";
            item2["type"] = "nav_point";
            Json::Value pos3;
            pos3["x"] = 20.5;
            pos3["y"] = 12.3;
            pos3["angle"] = 0.1;
            item3["pos"] = pos3;
            waypoints.append(item3);

            element["waypoints"] = waypoints;
            data["element"] = element;

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string strData = Json::writeString(builder, root);

            msgsend_->m_proHeader.m_length = strData.length();
            msgsend_->m_strJsonBody = strData;
            break;
        }

        case CMD_REQ[CmdConfigDeleteMap]:
        {
            msgsend_->m_proHeader.m_length = 0;
            break;
        }

        }
    }
};
