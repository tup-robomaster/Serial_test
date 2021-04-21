cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Debug") # "Debug" "Release"
PROJECT(Serial)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
ADD_EXECUTABLE(Serial ./main.cpp
                    ./CRC_Check.cpp
                    ./CRC_Check.h
                    ./serialport.cpp
                    ./serialport.h
)