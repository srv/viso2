cmake_minimum_required(VERSION 2.8.3)
project(viso2_ros)
set(CMAKE_MESSAGE_LOG_LEVEL "STATUS" )

set(OpenCV_DIR /usr/local/share/OpenCV)
set(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" TRUE)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

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

add_message_files( DIRECTORY msg FILES VisoInfo.msg)

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

add_executable(testORB src/testORB.cpp) # BMNF(04/01/2021): MODIFIED
add_dependencies(testORB ${catkin_EXPORTED_TARGETS}) # BMNF(04/01/2021): MODIFIED 
add_dependencies(testORB ${PROJECT_NAME}_gencpp) # BMNF(04/01/2021): MODIFIED 

add_executable(testSIFT4 src/testSIFT4.cpp) # BMNF(16/04/2021): NEW 
add_dependencies(testSIFT4 ${catkin_EXPORTED_TARGETS}) # BMNF(16/04/2021): NEW 
add_dependencies(testSIFT4 ${PROJECT_NAME}_gencpp) # BMNF(16/04/2021): NEW 

add_executable(testFAST1 src/testFAST1.cpp) # BMNF(03/06/2021): NEW 
add_dependencies(testFAST1 ${catkin_EXPORTED_TARGETS}) # BMNF(03/06/2021): NEW 
add_dependencies(testFAST1 ${PROJECT_NAME}_gencpp) # BMNF(03/06/2021): NEW 

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})

target_link_libraries(stereo_odometer ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer_omnidirectional ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(testORB ${OpenCV_LIB} ${catkin_LIBRARIES}) # BMNF(16/11/2020): MODIFIED
target_link_libraries(testSIFT4 ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}) # BMNF(16/04/2021): NEW 
target_link_libraries(testFAST1 ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}) # BMNF(03/06/2021): NEW 






