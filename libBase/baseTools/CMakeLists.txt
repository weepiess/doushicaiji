include_directories(./include)

AUX_SOURCE_DIRECTORY(./src BASETOOLS_SRCS)
ADD_LIBRARY(BaseTools ${BASETOOLS_SRCS})
