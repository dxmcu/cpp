#include "request_handler.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
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
    rep.content = "1324134315";
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

} // namespace server
} // namespace http
