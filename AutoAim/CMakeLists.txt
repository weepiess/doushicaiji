include_directories(./include)
include_directories(${PROJECT_SOURCE_DIR} /libAutoAim/include)
AUX_SOURCE_DIRECTORY(./src AUTOAIM_SRCS)
ADD_LIBRARY(AutoAim ${AUTOAIM_SRCS})
