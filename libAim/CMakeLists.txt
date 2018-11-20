add_subdirectory(infantryAutoAim)

include_directories(./include)
AUX_SOURCE_DIRECTORY(./src BASEAIM_SRCS)
ADD_LIBRARY(BaseAim ${BASEAIM_SRCS})
