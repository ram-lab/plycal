if( CMAKE_BUILD_TYPE EQUAL "Release")
	add_definitions( -DNODEBUG )
endif(CMAKE_BUILD_TYPE EQUAL "Release")


add_executable(
	test_image_polygon
	test_image_polygon.cpp
	)
target_link_libraries(
	test_image_polygon
	plycal
	)

add_executable(
	test_image_polygons
	test_image_polygons.cpp
	)
target_link_libraries(
	test_image_polygons
	plycal
	)

add_executable(
	test_pointcloud_polygon
	test_pointcloud_polygon.cpp
	)
target_link_libraries(
	test_pointcloud_polygon
	plycal
	)


