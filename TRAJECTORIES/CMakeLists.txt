cmake_minimum_required(VERSION 3.2)

include_directories(./include ./src ./data_plot)

project(TRAJECTORY)

add_executable(trajectoryTest test.cpp src/polynomialTraj.cpp)
