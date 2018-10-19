include_directories(./include)

AUX_SOURCE_DIRECTORY(./src AUTOAIM_SRCS)
ADD_LIBRARY(libAutoAim ${AUTOAIM_SRCS})

