cmake_minimum_required(VERSION 3.5)
project(control)

add_executable(${PROJECT_NAME} main.cpp)

set(CMAKE_CXX_STANDARD 11)

option(RUN_X11_TESTS "Run tests that require X11" OFF)

#Mujoco paths
include_directories(${CMAKE_SOURCE_DIR}/../include)
link_directories(${CMAKE_SOURCE_DIR}/../lib)

file(GLOB LIB_MUJOCO ${CMAKE_SOURCE_DIR}/../lib/libmujoco.so)

message(STATUS "MuJoCo lib: " ${LIB_MUJOCO})

target_link_libraries(${PROJECT_NAME} ${LIB_MUJOCO})