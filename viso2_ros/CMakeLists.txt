cmake_minimum_required(VERSION 3.0.2)
project(viso2_ros)
set(CMAKE_MESSAGE_LOG_LEVEL "STATUS" )

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)
set(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" TRUE)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  libviso2
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  image_transport
  cv_bridge
  image_geometry
  tf
  pcl_ros
  std_srvs
  message_generation
)

link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL REQUIRED)

option(ARM_CROSS_COMPILATION "ARM Cross Compilation" OFF)

add_message_files(DIRECTORY msg FILES VisoInfo.msg)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package()

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  SET(CMAKE_CXX_FLAGS -mfpu=neon)
else(ARM_CROSS_COMPILATION)
  add_definitions(-msse3)
endif(ARM_CROSS_COMPILATION)

include_directories(src ${libviso2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(stereo_odometer src/stereo_odometer.cpp)
add_dependencies(stereo_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(stereo_odometer ${PROJECT_NAME}_gencpp)

add_executable(mono_odometer src/mono_odometer.cpp)
add_dependencies(mono_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(mono_odometer ${PROJECT_NAME}_gencpp)

add_executable(mono_odometer_omnidirectional src/mono_odometer_omnidirectional.cpp)
add_dependencies(mono_odometer_omnidirectional ${catkin_EXPORTED_TARGETS})
add_dependencies(mono_odometer_omnidirectional ${PROJECT_NAME}_gencpp)

add_executable(get_first_point src/get_first_point.cpp) # BMNF(07/04/2022): NEW 
add_dependencies(get_first_point ${catkin_EXPORTED_TARGETS}) # BMNF(07/04/2022): NEW 
add_dependencies(get_first_point ${PROJECT_NAME}_gencpp) # BMNF(07/04/2022): NEW 

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

target_link_libraries(stereo_odometer ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer_omnidirectional ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(get_first_point ${Boost_LIBRARIES} ${catkin_LIBRARIES}) # BMNF(07/04/2022): NEW 






