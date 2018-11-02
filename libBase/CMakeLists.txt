include_directories(./include)
AUX_SOURCE_DIRECTORY(./src BASE_SRCS)
ADD_LIBRARY(Base ${BASE_SRCS})

