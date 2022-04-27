cmake_minimum_required(VERSION 2.8)
project(icp_lidar)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

find_package(PythonLibs REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    "/usr/include/eigen3"
    "/usr/local/include/knn"
    ${PYTHON_INCLUDE_DIRS}
    "/usr/local/lib/python3.6/dist-packages/numpy/core/include"
)

add_executable(icp_lidar src/icp_lidar.cpp src/icp_lidar_main.cpp)
target_link_libraries(icp_lidar ${PYTHON_LIBRARIES})