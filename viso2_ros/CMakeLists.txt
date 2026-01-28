cmake_minimum_required(VERSION 3.0.2)
project(viso2_ros)


##############
## Compiler ##
##############

# C++17 Standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags.
add_compile_options(-Wall)

option(ARM_CROSS_COMPILATION "ARM Cross Compilation" OFF)

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  add_definitions(-mfpu=neon)
else()
  add_definitions(-msse3)
endif(ARM_CROSS_COMPILATION)


#######################
## Required packages ##
#######################

find_package(catkin REQUIRED COMPONENTS
  libviso2
  
  tf2
  roscpp
  tf2_ros
  pcl_ros
  std_srvs
  std_msgs
  nav_msgs
  cv_bridge
  sensor_msgs
  image_geometry
  message_filters
  image_transport
  tf2_geometry_msgs
  message_generation
)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)

# # Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - Boost
find_package(Boost REQUIRED COMPONENTS thread)


###################################
## Catkin specific configuration ##
###################################

# Declare custom ROS messages
add_message_files( 
  DIRECTORY msg 
  FILES VisoInfo.msg
)

# Declare custom ROS services
add_service_files(
    DIRECTORY srv
    FILES GetTransform.srv
)

# Generate custom ROS messages
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp 
    libviso2 
    std_msgs
    sensor_msgs
    message_runtime
)


###########
## Build ##
###########

include_directories(
  include 
  ${libviso2_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

# Executables.
add_executable(initial_pose_server_node 
               src/initial_pose_server.cpp
               src/initial_pose_server_node.cpp)
add_executable(mono_odometer 
               src/mono_odometer.cpp)
add_executable(mono_odometer_omnidirectional 
               src/mono_odometer_omnidirectional.cpp)
add_executable(stereo_odometer 
               src/stereo_odometer.cpp)

# Dependencies.
add_dependencies(initial_pose_server_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(mono_odometer ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(mono_odometer_omnidirectional ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(stereo_odometer ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Linking.
target_link_libraries(initial_pose_server_node ${catkin_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer_omnidirectional ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})