cmake_minimum_required(VERSION 3.5)
project(slam_project)
set (CMAKE_CXX_STANDARD 11)

include_directories(include)
find_package(OpenCV REQUIRED)

add_executable(proj
    src/main.cpp
    src/MapReader.cpp
    src/node.cpp
)

set(CMAKE_BINARY_DIR "../bin")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

target_link_libraries(proj)
target_link_libraries(proj PUBLIC ${OpenCV_LIBS})
