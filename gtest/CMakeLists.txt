cmake_minimum_required(VERSION 2.8)

project(abs_test)
add_executable(${PROJECT_NAME} "abs_test.cc")

SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/gtest")
find_package(GTest REQUIRED)

MESSAGE(${GTEST_MAIN_LIBRARIES})
MESSAGE(${GTEST_INCLUDE_DIRS})
MESSAGE(${GTEST_LIBRARIES})

include_directories(${GTEST_INCLUDE_DIRS})

target_link_libraries(abs_test rt)
target_link_libraries(abs_test pthread)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GTEST_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)
