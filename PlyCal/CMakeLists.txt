if( CMAKE_BUILD_TYPE EQUAL "Release")
	add_definitions( -DNODEBUG )
endif(CMAKE_BUILD_TYPE EQUAL "Release")

file(GLOB LIB_SRC
    "*.cpp"
	)
file(GLOB LIB_HEADER
    "*.h"
	)
find_package(
	OpenCV
	REQUIRED
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
find_package(
	Boost
	REQUIRED
	COMPONENTS
	system
	thread
	)
find_package(
	Eigen3
	REQUIRED
	)
# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DEFINITIONS})

include_directories(
	${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	)
link_directories(
	${PCL_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
	)

set(
	EDLINE_SRC
	${PROJECT_SOURCE_DIR}/thirdParty/edlines/EDLineDetector.cpp
	)


add_library(
	plycal
	SHARED
	${LIB_SRC}
	${LIB_HEADER}
	${EDLINE_SRC}
	)
target_link_libraries(
	plycal
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
	${CERES_LIBRARIES}
	)
set_target_properties(
	plycal
	PROPERTIES
	OUTPUT_NAME
	${LIB_PREFIX}
	)
