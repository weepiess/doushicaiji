cmake_minimum_required(VERSION 3.5)

project(zimiao)
set(OpenCV_DIR /usr/share/opencv)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libHardWare/serialPort/include)
include_directories(${PROJECT_SOURCE_DIR}/libHardWare/usbCapture/include)
include_directories(${PROJECT_SOURCE_DIR}/libAim/include)
include_directories(${PROJECT_SOURCE_DIR}/libAim/autoAim/include)
include_directories(${PROJECT_SOURCE_DIR}/libAim/markAim/include)
include_directories(${PROJECT_SOURCE_DIR}/libBase/baseClasses/include)
include_directories(${PROJECT_SOURCE_DIR}/libBase/baseTools/include)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libBase)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libHardWare)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/libAim)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

add_executable(example ${DIR_SRCS})

target_link_libraries(example SerialPort UsbCapture)
target_link_libraries(example pthread)
target_link_libraries(example AutoAim MarkAim)
target_link_libraries(example BaseClasses BaseTools)

find_package(OpenCV 3.4.3 REQUIRED)
target_link_libraries(example ${OpenCV_LIBS})
