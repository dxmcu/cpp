cmake_minimum_required(VERSION 3.0)

project(Robot)
link_directories("/usr/local/bluez/lib")
include_directories("/usr/local/bluez/include")

#add_definitions(-DBOOST_LOG_DYN_LINK)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(simplescan simplescan.c)
target_link_libraries(simplescan bluetooth)

add_executable(rfcomm_server rfcomm-server.c)
target_link_libraries(rfcomm_server bluetooth)

add_executable(rfcomm_client rfcomm-client.c)
target_link_libraries(rfcomm_client bluetooth)
