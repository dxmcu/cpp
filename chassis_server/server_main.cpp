#include "SeerServer.h"

int main(int argc, char* argv[])
{
    boost::asio::io_service io_service;
    tcp::endpoint endpoint(address::from_string("127.0.0.1"), 19204);
    tcp::endpoint endpoint2(address::from_string("127.0.0.1"), 19205);

    Server s(io_service, endpoint, endpoint2);
    s.run();
    return 0;
}
