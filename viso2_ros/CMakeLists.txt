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
  pcl_ros
  std_srvs
  message_generation
)

set(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" TRUE)
set(OpenCV_INCLUDE_DIRS
  /usr/local/include
  /usr/local/include/opencv2
)

set(OpenCV_LIB_DIR
  /usr/local/lib
)

set(OpenCV_LIBS
  opencv_calib3d
  opencv_core
  opencv_features2d
  opencv_flann
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_ml
  opencv_objdetect
  opencv_photo
  opencv_shape
  opencv_stitching
  opencv_superres
  opencv_video
  opencv_videoio
  opencv_videostab
  opencv_viz
  opencv_aruco
  opencv_bgsegm
  opencv_bioinspired
  opencv_ccalib
  opencv_datasets
  opencv_dpm
  opencv_face
  opencv_freetype
  opencv_fuzzy
  opencv_hdf
  opencv_line_descriptor
  opencv_optflow
  opencv_phase_unwrapping
  opencv_plot
  opencv_reg
  opencv_rgbd
  opencv_saliency
  opencv_stereo
  opencv_structured_light
  opencv_surface_matching
  opencv_text
  opencv_ximgproc
  opencv_xobjdetect
  opencv_xphoto
 opencv_xfeatures2d
)

set(OpenCV_LIBRARIES
  opencv_calib3d
  opencv_core
  opencv_features2d
  opencv_flann
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_ml
  opencv_objdetect
  opencv_photo
  opencv_shape
  opencv_stitching
  opencv_superres
  opencv_video
  opencv_videoio
  opencv_videostab
  opencv_viz
  opencv_aruco
  opencv_bgsegm
  opencv_bioinspired
  opencv_ccalib
  opencv_datasets
  opencv_dpm
  opencv_face
  opencv_freetype
  opencv_fuzzy
  opencv_hdf
  opencv_line_descriptor
  opencv_optflow
  opencv_phase_unwrapping
  opencv_plot
  opencv_reg
  opencv_rgbd
  opencv_saliency
  opencv_stereo
  opencv_structured_light
  opencv_surface_matching
  opencv_text
  opencv_ximgproc
  opencv_xobjdetect
  opencv_xphoto
 opencv_xfeatures2d
)



#find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS})
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

add_executable(testSIFT src/testSIFT.cpp) # BMNF(30/11/2020): NEW 
add_dependencies(testSIFT ${catkin_EXPORTED_TARGETS}) # BMNF(30/11/2020): NEW 
add_dependencies(testSIFT ${PROJECT_NAME}_gencpp) # BMNF(30/11/2020): NEW 

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer_omnidirectional ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(testORB ${OpenCV_LIB} ${catkin_LIBRARIES}) # BMNF(16/11/2020): MODIFIED
target_link_libraries(testSIFT ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}) # BMNF(30/11/2020): NEW






