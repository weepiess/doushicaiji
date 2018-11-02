
#project(deepcam_test)
#set(OpenCV_DIR /usr/local/share/openCV)
#set(CMAKE_CXX_STANDARD 11)

#link_directories(${CAMDRIVER_LIBRARY_DIRS})
#add_definitions(${CAMDRIVER_DEFINITIONS})

include_directories($ENV{OPENNI2_INCLUDE})
link_directories($ENV{OPENNI2_REDIST})

#find_package(PCL 1.8 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#find_package(OpenCV 3.4.3 REQUIRED)
#include_directories(${PROJECT_SOURCE_DIR}/include)
#AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
#add_executable (deepcam ${DIR_SRCS})
#target_link_libraries(deepcam ${OpenCV_LIBS} libOpenNI2.so )


include_directories(./include)
AUX_SOURCE_DIRECTORY(./src DEEPCAM_SRCS)
ADD_LIBRARY(deepcam ${DEEPCAM_SRCS})
