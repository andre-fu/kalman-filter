cmake_minimum_required(VERSION 3.10)
project(kalman-filter)
add_executable(kalman kalman-main.cpp)

set(EIGEN_INC_DIR /usr/include/eigen3/)
include_directories(${EIGEN_INC_DIR})

set(CMAKE_CXX_STANDARD 14)	

