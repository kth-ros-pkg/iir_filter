cmake_minimum_required(VERSION 2.8.3)
project(iir_filter)

find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
  LIBRARIES iir_filter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(iir_filter src/iir_filter.cpp)

install(TARGETS iir_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
