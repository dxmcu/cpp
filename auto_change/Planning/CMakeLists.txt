cmake_minimum_required(VERSION 3.0)
project(planning)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(/usr/include/eigen3)
include_directories(/usr/local/glog/include)
link_directories(/usr/local/glog/lib)

add_executable(planning planning.cpp)

target_sources(planning PRIVATE AlgExcTimer.cpp)
target_sources(planning PRIVATE cost_select.cpp)
target_sources(planning PRIVATE loghelp.cpp)
target_sources(planning PRIVATE route_planner.cpp)
target_sources(planning PRIVATE speed_planner.cpp)

target_link_libraries(planning rt)
target_link_libraries(planning pthread)
target_link_libraries(planning jsoncpp)
target_link_libraries(planning glog)
target_link_libraries(planning zmq)
target_link_libraries(planning config)
target_link_libraries(planning boost_system)
target_link_libraries(planning boost_filesystem)

