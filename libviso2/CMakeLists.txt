cmake_minimum_required(VERSION 2.8.3)
project(libviso2)

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  cv_bridge
)

option(ARM_CROSS_COMPILATION "ARM Cross Compilation" OFF)

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  SET(CMAKE_CXX_FLAGS -mfpu=neon)
else(ARM_CROSS_COMPILATION)
  add_definitions(-msse3)
endif(ARM_CROSS_COMPILATION)

catkin_package(
  INCLUDE_DIRS libviso2/src
  LIBRARIES viso2
)

set(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" TRUE)

if(OPENCV_ENABLE_NONFREE)
    message("  Non-free algorithms are enabled")
else()
    message("  Non-free algorithms are NOT enabled")
endif()


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

include_directories(libviso2/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})

#TARGET_LINK_LIBRARIES(libviso2 ${OpenCV_LIBS})

add_library(viso2
  libviso2/src/filter.cpp
  libviso2/src/matcher.cpp
  libviso2/src/matrix.cpp
  libviso2/src/reconstruction.cpp
  libviso2/src/triangle.cpp
  libviso2/src/viso.cpp
  libviso2/src/viso_mono.cpp
  libviso2/src/viso_mono_omnidirectional.cpp
  libviso2/src/viso_stereo.cpp
)

target_link_libraries(viso2 ${OpenCV_LIB} ${OpenCV_LIBRARIES})

install(TARGETS viso2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_NAME}/src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".cpp" EXCLUDE
)

