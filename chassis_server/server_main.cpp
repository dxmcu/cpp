#include "chassis_server.h"

int main(int /*argc*/, char* /*argv*/[])
{
    boost::asio::io_service io_service;
    tcp::endpoint endpoint(address::from_string("127.0.0.1"), 11235);
    tcp::endpoint endpoint2(address::from_string("127.0.0.1"), 11236);
    tcp::endpoint endpoint3(address::from_string("127.0.0.1"), 11237);
    tcp::endpoint endpoint4(address::from_string("127.0.0.1"), 11238);

    Server s(io_service, endpoint, endpoint2, endpoint3, endpoint4);
    s.run();
    return 0;
}
