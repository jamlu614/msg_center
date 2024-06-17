cmake_minimum_required(VERSION 3.10.0)
project(msg_center VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/metro)
link_directories(${CMAKE_SOURCE_DIR}/metro)

include_directories(proto)
file(GLOB proto_src proto/*cc)

# 查找 Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(msg_center main.cpp ${proto_src})
target_link_libraries(
    msg_center

    pthread 
    gac_ipc_metro
    boost_system
    rt
    mosquitto
    inih
    protobuf
)

