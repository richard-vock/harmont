cmake_minimum_required(VERSION 2.8.8)
project(harmont)

set (harmont_VERSION_MAJOR 0)
set (harmont_VERSION_MINOR 1)
set (OpenGL_GL_PREFERENCE GLVND)

configure_file (
  "${PROJECT_SOURCE_DIR}/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/config.hpp"
)

include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
    if (NOT WIN32)
        add_definitions(-fPIC)
    else()
        add_definitions(-DM_LOG2E=1.44269504088896340736)
    endif()
	add_definitions(-O3)
	add_definitions(-g)
    add_definitions(-Wall)
endif()

find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(OpenGL)
find_package(GLEW)
include_directories(${GLEW_INCLUDE_DIRS})

find_package(Boost COMPONENTS system regex)
include_directories(${Boost_INCLUDE_DIRS})

find_package(GLUT)
if (GLUT_FOUND)
    add_definitions(-DUSE_FREEGLUT)
    include_directories(${GLUT_INCLUDE_DIRS})
else()
    message(STATUS "No freeglut package found. Freeglut support will be disabled.")
    set(GLUT_LIBRARIES "")
endif()

find_package(PCL COMPONENTS common io)
if (PCL_FOUND)
    add_definitions(-DUSE_PCL)
    include_directories(${PCL_INCLUDE_DIRS})
else()
    message(STATUS "No PCL package found. PCL support will be disabled.")
    set(PCL_LIBRARIES "")
endif()
# fix for broken PCL build system
find_package(Boost COMPONENTS system regex)
add_definitions(-DBUILD_DEFERRED_RENDERER)


file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp" "src/rgbe.c")
if (GLEW_FOUND AND EIGEN_FOUND)
	add_library(harmont SHARED ${obj})
    set_property(TARGET harmont PROPERTY CXX_STANDARD 17)
	target_link_libraries(harmont ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${PCL_LIBRARIES} "dl" "stdc++fs")

    add_executable(pcl_deferred "${PROJECT_SOURCE_DIR}/apps/pcl_deferred.cpp")
    set_property(TARGET pcl_deferred PROPERTY CXX_STANDARD 17)
    target_link_libraries(pcl_deferred harmont ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} "GL")

    add_executable(compute_normals "${PROJECT_SOURCE_DIR}/apps/compute_normals.cpp")
    set_property(TARGET compute_normals PROPERTY CXX_STANDARD 17)
    target_link_libraries(compute_normals ${PCL_LIBRARIES} ${Boost_LIBRARIES} "GL" "stdc++fs")
endif()

