# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)

# directories
set (LIBVISO2_SRC_DIR src)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  SET(CMAKE_CXX_FLAGS -mfpu=neon)
elseif (ARM64_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR aarch64)
  add_definitions(-D__ARM_NEON__)
else()
  add_definitions(-msse3)
endif()

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(viso2 ${LIBVISO2_SRC_FILES})
target_link_libraries (viso2 png)

