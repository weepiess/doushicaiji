include_directories(./include)
#依赖基本库
include_directories(${PROJECT_SOURCE_DIR}/libBase/include)
AUX_SOURCE_DIRECTORY(./src USBCAPTURE_DIR_SRCS)

ADD_LIBRARY(UsbCapture ${USBCAPTURE_DIR_SRCS})
target_link_libraries(UsbCapture Base)





