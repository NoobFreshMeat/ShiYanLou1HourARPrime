set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.8)

project( marker )

find_package( OpenCV REQUIRED )

add_executable( marker marker.cpp )
target_link_libraries( marker ${OpenCV_LIBS} )

