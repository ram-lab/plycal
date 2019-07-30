set(CMAKE_INCLUDE_CURRENT_DIR ON)
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI file


find_package(
	Qt5
	REQUIRED
	COMPONENTS
	Core
	Gui
	Widgets
	)
find_package(
	VTK
	REQUIRED
	)

include_directories(
	${VTK_INCLUDE_DIRS}
	)

link_directories(
	${VTK_LIBRARY_DIRS}
	)

file(GLOB UI_SRC
    "*.cpp"
)

add_executable(
	${PROJECT_NAME}-qt
	${UI_SRC}
	)

target_link_libraries(
	${PROJECT_NAME}-qt
	${VTK_LIBRARIES}
	${LIB_PREFIX}
	Qt5::Core
	Qt5::Widgets
	Qt5::Gui
	)

