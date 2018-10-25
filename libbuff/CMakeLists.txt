include_directories(./include)
AUX_SOURCE_DIRECTORY(./src BUFF_SRCS)
ADD_LIBRARY(libbuff ${BUFF_SRCS})