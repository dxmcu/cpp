#include "request_handler.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <jsoncpp/json/json.h>
#include "mime_types.hpp"
#include "reply.hpp"
#include "request.hpp"

namespace http {
namespace server {

request_handler::request_handler(const std::string& doc_root)
    : doc_root_(doc_root)
{
}//构造函数

void request_handler::handle_request(const request& req, reply& rep)
{//解析请求

    if (req.uri == "/gs-robot/cmd/is_initialize_finished")
    {
        Json::Value root;
        root["data"] = "true";
        root["errorCode"] = "";
        root["msg"] = "successed";
        root["successed"] = true;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        rep.content = Json::writeString(builder, root);
    }

    else if (req.uri == "/gs-robot/data/device_status")
    {
        Json::Value root;
        Json::Value data;
        data["battery"] = 45.435484000000002;
        data["batteryVoltage"] = 21.815000000000001;
        data["charge"] = false;
        data["charger"] = 0;
        data["chargerCurrent"] = 0.604688;
        data["chargerStatus"] = false;
        data["chargerVoltage"] = 0.021000000000000001;
        data["emergency"] = false;
        data["emergencyStop"] = false;
        data["indexUpdatedAt"] = 1515901694;
        data["navigationSpeedLevel"] = 1;
        data["playPathSpeedLevel"] = 2;
        data["speed"] = 0;
        data["statusUpdatedAt"] = 1515896014;
        data["totalMileage"] = 0;
        data["uptime"] = 5709;
        root["data"] = data;
        root["errorCode"] = "";
        root["msg"] = "successed";
        root["successed"] = true;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        rep.content = Json::writeString(builder, root);
    }

    else if (req.uri == "/gs-robot/real_time_data/cmd_vel")
    {
        Json::Value root;
        Json::Value data, angular, linear;
        angular["x"] = 0;
        angular["y"] = 0;
        angular["z"] = -0.1;
        linear["x"] = 0.2;
        linear["y"] = 0;
        linear["z"] = 0;
        data["angular"] = angular;
        data["linear"] = linear;
        root["data"] = data;
        root["errorCode"] = "";
        root["msg"] = "successed";
        root["successed"] = true;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        rep.content = Json::writeString(builder, root);
    }

    else if (req.uri == "/gs-robot/real_time_data/position")
    {
        Json::Value root;
        root["angle"] = -173.41528128678252;
        Json::Value gridPosition;
        gridPosition["x"] = 372;
        gridPosition["y"] = 502;
        root["gridPosition"] = gridPosition;
        Json::Value mapInfo;
        mapInfo["gridHeight"] = 992;
        mapInfo["gridWidth"] = 992;
        mapInfo["originX"] = -24.8;
        mapInfo["originY"] = -24.8;
        mapInfo["resolution"] = 0.05000000074505806;
        root["mapInfo"] = mapInfo;

        Json::Value worldPosition, orientation, position;
        orientation["w"] = -0.05743089347363588;
        orientation["x"] = 0;
        orientation["y"] = 0;
        orientation["z"] = 0.9983494841361015;
        worldPosition["orientation"] = orientation;
        position["x"] = -6.189813393986145;
        position["y"] = 0.3017086724551712;
        position["z"] = 0;
        worldPosition["position"] = position;
        root["worldPosition"] = worldPosition;

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        rep.content = Json::writeString(builder, root);
    }

    else if (req.uri == "/gs-robot/data/maps")
    {
        Json::Value root, data;
        Json::Value item;
        item["createdAt"] = this->CurrentTime();
        item["dataFileName"] = "40dd8fcd-5e6d-4890-b620-88882d9d3977.data";
        item["id"] = 0;
        Json::Value mapInfo;
        mapInfo["gridHeight"] = 992;
        mapInfo["gridWidth"] = 992;
        mapInfo["originX"] = -24.8;
        mapInfo["originY"] = -24.8;
        mapInfo["resolution"] = 0.05;
        item["mapInfo"] = mapInfo;
        item["name"] = "demo";
        item["obstacleFileName"] = "";
        item["pgmFileName"] = "6a3e7cae-c4a8-4583-9a5d-08682344647a.pgm";
        item["pngFileName"] = "228b335f-8c1a-4f05-a292-160f942cbe00.png";
        item["yamlFileName"] = "4108be8c-4004-4ad6-a9c5-599b4a3d49df.yaml";
        data.append(item);
        Json::Value item2;
        item2["createdAt"] = "2016-07-27 23:37:31";
        item2["dataFileName"] = "df5ff3c6-ac5c-4365-a89a-ca0128057006.data";
        item2["id"] = 0;
        Json::Value mapInfo2;
        mapInfo2["gridHeight"] = 992;
        mapInfo2["gridWidth"] = 992;
        mapInfo2["originX"] = -24.8;
        mapInfo2["originY"] = -24.8;
        mapInfo2["resolution"] = 0.05;
        item2["mapInfo"] = mapInfo2;
        item2["name"] = "tom5";
        item2["obstacleFileName"] = "";
        item2["pgmFileName"] = "8768e979-6a27-46db-a479-b729036970b3.pgm";
        item2["pngFileName"] = "5ef8bd27-5ffa-4c44-8f25-2f2811d0d2e8.png";
        item2["yamlFileName"] = "e2c0cc20-6ee8-4ce9-91c0-1fe1fce39857.yaml";
        data.append(item2);
        root["data"] = data;
        root["errorCode"] = "";
        root["msg"] = "successed";
        root["successed"] = true;

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        rep.content = Json::writeString(builder, root);
    }

    //响应头
    rep.headers.resize(2);
    rep.headers[0].name = "Content-Length";
    rep.headers[0].value = std::to_string(rep.content.size());
    rep.headers[1].name = "Content-Type";
    rep.headers[1].value = mime_types::extension_to_type("html");//扩展名->Content-Type
}

bool request_handler::url_decode(const std::string& in, std::string& out)
{
    out.clear();
    out.reserve(in.size());//
    for (std::size_t i = 0; i < in.size(); ++i)
    {
        if (in[i] == '%')//转义字符
        {
            if (i + 3 <= in.size())
            {
                int value = 0;
                std::istringstream is(in.substr(i + 1, 2));
                if (is >> std::hex >> value)//16进制转10进制（0-255）
                {
                    out += static_cast<char>(value);
                    i += 2;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else if (in[i] == '+')//+表示空格
        {
            out += ' ';
        }
        else//如果非特殊符号直接
        {
            out += in[i];
        }
    }
    return true;
}//解析url

std::string request_handler::CurrentTime()
{
    std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm* ptm = localtime(&tt);

    std::stringbuf buf;
    std::ostream os(&buf);
    os << ptm->tm_year + 1900 << "-" << ptm->tm_mon + 1 << "-" << ptm->tm_mday << " " <<
          ptm->tm_hour << ":" << ptm->tm_min << ";" << ptm->tm_sec;
    return buf.str();
}


} // namespace server
} // namespace http
