cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS, "-g")

project(minco)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(Eigen3_INCLUDE_DIRS "/usr/include/eigen3")
SET(Python3_INCLUDE_DIRS "/usr/include/python3.8")
SET(Python3_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS} 
  ${Python3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/test.cpp)

target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})
