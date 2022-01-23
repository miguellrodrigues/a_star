cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 14)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

find_package(OpenCV REQUIRED)

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)

set(SOURCES ${C_SOURCES} ${CPP_SOURCES} main.cpp Cell.cpp Cell.h)

set(LIBRARIES ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES})