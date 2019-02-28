cmake_minimum_required(VERSION 2.8.8)
project(harmont)

set (OpenGL_GL_PREFERENCE GLVND)

include_directories("${PROJECT_SOURCE_DIR}/include")

configure_file (
  "${PROJECT_SOURCE_DIR}/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/config.hpp"
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
add_executable(cloud_renderer ${obj} "${PROJECT_SOURCE_DIR}/apps/cloud_renderer.cpp")
target_link_libraries(cloud_renderer Eigen3::Eigen GLUT::GLUT GLEW::GLEW OpenGL::OpenGL)

#add_executable(compute_normals "${PROJECT_SOURCE_DIR}/apps/compute_normals.cpp")
#set_property(TARGET compute_normals PROPERTY CXX_STANDARD 17)
#target_link_libraries(compute_normals ${PCL_LIBRARIES} ${Boost_LIBRARIES} "GL" "stdc++fs")
