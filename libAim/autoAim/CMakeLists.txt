include_directories(./include)

AUX_SOURCE_DIRECTORY(./src AUTOAIM_SRCS)
ADD_LIBRARY(AutoAim ${AUTOAIM_SRCS})
target_link_libraries(AutoAim BaseAim)
