cmake_minimum_required(VERSION 2.8.3)
project(perception_fusion)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

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

target_link_libraries(perception_fusion ${catkin_LIBRARIES})
