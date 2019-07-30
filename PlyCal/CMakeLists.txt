if( CMAKE_BUILD_TYPE EQUAL "Release")
	add_definitions( -DNODEBUG )
endif(CMAKE_BUILD_TYPE EQUAL "Release")

file(GLOB LIB_SRC
    "*.cpp"
)
file(GLOB LIB_HEADER
    "*.h"
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