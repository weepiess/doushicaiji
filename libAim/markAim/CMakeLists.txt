include_directories(./include)

AUX_SOURCE_DIRECTORY(./src MARKAIM_SRCS)
ADD_LIBRARY(MarkAim ${MARKAIM_SRCS})