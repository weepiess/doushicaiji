include_directories(./include)

AUX_SOURCE_DIRECTORY(./src TOOLS_SRCS)
ADD_LIBRARY(Tools ${TOOLS_SRCS})
