cmake_minimum_required(VERSION 3.0)

project(cmakecreator)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(/usr/local/boost-1.68/include)
link_directories(/usr/local/boost-1.68/lib)

add_executable(${PROJECT_NAME} main.cpp)
#target_sources(${PROJECT_NAME}  PRIVATE test.cpp)
target_link_libraries(${PROJECT_NAME} rt)
target_link_libraries(${PROJECT_NAME} pthread)
target_link_libraries(${PROJECT_NAME} boost_system)
