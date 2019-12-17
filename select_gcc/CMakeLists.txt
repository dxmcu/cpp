cmake_minimum_required(VERSION 2.8)

SET(CMAKE_C_COMPILER /home/lz/bin/gcc-9.2.0/bin/gcc)
SET(CMAKE_CXX_COMPILER /home/lz/bin/gcc-9.2.0/bin/g++)

add_compile_options(-std=c++17)

project(select_gcc)
add_executable(${PROJECT_NAME} "main.cc")
