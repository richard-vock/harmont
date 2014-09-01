cmake_minimum_required(VERSION 2.8.8)
project(harmont)

set (harmont_VERSION_MAJOR 0)
set (harmont_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/inline")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-fPIC)
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++1y)
    add_definitions(-Wall)
endif()

find_package(Eigen)
find_package(OpenGL)
find_package(GLEW)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
file (GLOB_RECURSE units RELATIVE "${PROJECT_SOURCE_DIR}" "test/units/*.cpp")
if (GLEW_FOUND AND EIGEN_FOUND)
	include_directories(${GLEW_INCLUDE_DIRS})
	include_directories(${EIGEN_INCLUDE_DIRS})
	add_library(harmont SHARED ${obj})
	target_link_libraries(harmont ${GLEW_LIBRARIES} "dl")
#	add_definitions(-Dprotected=public)
#	add_definitions(-DTESTING)
#	add_executable(test "test/test.cpp" ${units})
#	target_link_libraries(test harmont ${GLEW_LIBRARIES} "dl" "pthread" "unittest" "damogran")

	# install binary
	install (TARGETS harmont DESTINATION lib)
	# install header
	install (DIRECTORY include/ DESTINATION include/harmont)
	install (DIRECTORY inline/ DESTINATION include/harmont)
endif()

