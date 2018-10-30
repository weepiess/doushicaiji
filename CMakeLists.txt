cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(deepcam)
set(OpenCV_DIR /usr/local/share/openCV)
set(CMAKE_CXX_STANDARD 11)

#link_directories(${CAMDRIVER_LIBRARY_DIRS})
#add_definitions(${CAMDRIVER_DEFINITIONS})
include_directories($ENV{OPENNI2_INCLUDE})
link_directories($ENV{OPENNI2_REDIST})
message($ENV{OPENNI2_INCLUDE})
message($ENV{OPENNI2_REDIST})

find_package(OpenCV 3.4.3 REQUIRED)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_executable (${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} libOpenNI2.so )

