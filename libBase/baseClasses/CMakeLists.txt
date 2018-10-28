include_directories(./include)

AUX_SOURCE_DIRECTORY(./src BASECLASSES_SRCS)
ADD_LIBRARY(BaseClasses ${BASECLASSES_SRCS})
