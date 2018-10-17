cmake_minimum_required(VERSION 3.5)
SET(OpenCV_DIR /usr/local/opencv/3.4.3/share/OpenCV)
project(zimiao)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

add_executable(example ${DIR_SRCS})

find_package(OpenCV REQUIRED)
target_link_libraries(example ${OpenCV_LIBS})
