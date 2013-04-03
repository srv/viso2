cmake_minimum_required(VERSION 2.8.3)
project(viso2_ros)

find_package(catkin REQUIRED COMPONENTS 
  libviso2 
  roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge 
  image_geometry 
  tf 
  pcl
  pcl_ros 
  std_srvs
)

find_package(OpenCV REQUIRED)

add_message_files( DIRECTORY msg FILES VisoInfo.msg)

add_executable(stereo_odometer
  src/stereo_odometer.cpp)

add_executable(mono_odometer
  src/mono_odometer.cpp)

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(stereo_odometer ${Boost_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIBRARIES})

include_directories(src ${libviso2_INCLUDE_DIRS} # WHY IS THIS NOT WORKING?
  ${catkin_INCLUDE_DIRS})

generate_messages(
  DEPENDENCIES 
    std_msgs
)

catkin_package(
    DEPENDS 
      opencv
    CATKIN-DEPENDS 
      libviso2 
      roscpp 
      sensor_msgs 
      nav_msgs 
      message_filters 
      image_transport 
      cv_bridge 
      image_geometry 
      tf 
      pcl
      pcl_ros 
      std_srvs
      message_runtime
)