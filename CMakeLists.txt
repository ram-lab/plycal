CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(PlyCal)

#cmake_policy(SET CMP0028 OLD)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++11) # for qt creator

# Optionally compiling part
option(BUILD_PlyCal_GUI "enables PlyCal viewer" true)
option(BUILD_PlyCal_TEST "enables PlyCal test" false)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

include_directories(
	thirdParty
	PlyCal
	)

# The library prefix
set(LIB_PREFIX plycal)
# Include the subdirectories
add_subdirectory(PlyCal)

if(BUILD_PlyCal_GUI)
	add_subdirectory(PlyCal_qt)
endif(BUILD_PlyCal_GUI)

if(BUILD_PlyCal_TEST)
	add_subdirectory(PlyCal_test)
endif(BUILD_PlyCal_TEST)


