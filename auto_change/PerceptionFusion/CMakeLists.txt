cmake_minimum_required(VERSION 3.0)
project(perception_fusion)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(/usr/include/eigen3)
include_directories(/usr/local/glog/include)
link_directories(/usr/local/glog/lib)

add_executable(perception_fusion src/PerceptionFusion.cpp)

target_sources(perception_fusion PRIVATE src/Fusioner.cpp)
target_sources(perception_fusion PRIVATE src/GeoUtil.cpp)
target_sources(perception_fusion PRIVATE src/MatchCost.cpp)
target_sources(perception_fusion PRIVATE src/VectorUtil.cpp)
target_sources(perception_fusion PRIVATE src/TrackObject.cpp)
target_sources(perception_fusion PRIVATE src/SensorData.cpp)
target_sources(perception_fusion PRIVATE src/Tracker.cpp)
target_sources(perception_fusion PRIVATE src/KalmanFilter.cpp)
target_sources(perception_fusion PRIVATE src/Geometry.cpp)

target_link_libraries(perception_fusion rt)
target_link_libraries(perception_fusion pthread)
target_link_libraries(perception_fusion jsoncpp)
target_link_libraries(perception_fusion glog)
target_link_libraries(perception_fusion zmq)
target_link_libraries(perception_fusion config)
target_link_libraries(perception_fusion boost_system)
target_link_libraries(perception_fusion boost_filesystem)

