CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(PlyCal)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++11) # for qt creator

# Optionally compiling part
option(BUILD_PlyCal_GUI "enables PlyCal viewer" true)
option(BUILD_PlyCal_TEST "enables PlyCal test" false)

# The library prefix
set(LIB_PREFIX plycal)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)


# set the output directory for the build executables and libraries
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(
  OpenCV
  REQUIRED
  )
find_package(
  Boost
  REQUIRED
  COMPONENTS
  system
  thread
  )
find_package(
  PCL
  QUIET
  REQUIRED
  )
find_package(
  Ceres
  REQUIRED
  )
# find_package(
#   Eigen
#   REQUIRED
#   )

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${Eigen_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  thirdParty
  PlyCal
  )
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  )

set(
	EDLINE_SRC  
	${PROJECT_SOURCE_DIR}/thirdParty/edlines/EDLineDetector.cpp
	)

if(BUILD_PlyCal_GUI)
 	add_subdirectory(PlyCal_qt)
endif(BUILD_PlyCal_GUI)

if(BUILD_PlyCal_TEST)
	add_subdirectory(PlyCal_test)
endif(BUILD_PlyCal_TEST)

# Include the subdirectories
add_subdirectory(PlyCal)
