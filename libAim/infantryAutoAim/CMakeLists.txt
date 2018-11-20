include_directories(./include)

AUX_SOURCE_DIRECTORY(./src INFANTRY_AUTOAIM_SRCS)
ADD_LIBRARY(InfantryAutoAim ${INFANTRY_AUTOAIM_SRCS})
target_link_libraries(InfantryAutoAim BaseAim)
