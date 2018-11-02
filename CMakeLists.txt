cmake_minimum_required(VERSION 3.5)

project(enginenear)
set(OpenCV_DIR /usr/local/share/openCV)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/libBase/include)
#include_directories(${PROJECT_SOURCE_DIR}/libHardWare/serialPort/include)
#include_directories(${PROJECT_SOURCE_DIR}/libTools/include)
#include_directories(${PROJECT_SOURCE_DIR}/deepcam/include)

#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libBase)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libHardWare)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libTools)
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/deepcam)


AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

add_executable(example ${DIR_SRCS})

#target_link_libraries(example Base)
#target_link_libraries(example SerialPort)
#target_link_libraries(example pthread)
#target_link_libraries(example Tools)
#target_link_libraries(example deepcam)

include_directories($ENV{OPENNI2_INCLUDE})
link_directories($ENV{OPENNI2_REDIST})

find_package(OpenCV 3.4.3 REQUIRED)
target_link_libraries(example ${OpenCV_LIBS} libOpenNI2.so)
