cmake_minimum_required(VERSION 2.8.3)
project(kf)


find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS UKF/include KF/include
  LIBRARIES kf
  CATKIN_DEPENDS
  DEPENDS Eigen
)

include_directories(kf/UKF/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


include_directories(kf/UKF/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(kf
  UKF/src/prediction.cpp
  KF/src/kalman_filter.cpp
  tools/src/tools.cpp)

target_link_libraries(kf
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES})


install(TARGETS kf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_NAME}/include
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
