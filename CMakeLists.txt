# This file based on the one from GLIDAR, and as such is under a BSD license.
# See: https://github.com/mohawkjohn/glidar

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(projected_area)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#list(APPEND CMAKE_CXX_FLAGS "-pthread -std=c++11 ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

if (APPLE)
  include (UseDebugSymbols)

  set(GUI_TYPE MACOSX_BUNDLE)

  find_library(OpenGL_LIBRARY OpenGL)
  find_library(Cocoa_LIBRARY Cocoa)
  find_library(IOKit_LIBRARY IOKit)
  find_library(CoreVideo_LIBRARY CoreVideo)

  mark_as_advanced(Cocoa_LIBRARY IOKit_LIBRARY CoreVideo_LIBRARY)

  set(EXTRA_LIBS ${Cocoa_LIBRARY} ${OpenGL_LIBRARY} ${IOKit_LIBRARY} ${CoreVideo_LIBRARY})
else (APPLE)
  find_package(OpenGL REQUIRED)
  include_directories(${OpenGL_INCLUDE_DIRS})
  link_directories(${OpenGL_LIBRARY_DIRS})
  add_definitions(${OpenGL_DEFINITIONS})
endif (APPLE)

find_package(ASSIMP REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
add_definitions(${ASSIMP_DEFINITIONS})

find_package(FLANN)
include_directories(${FLANN_INCLUDE_DIRS})

find_package(GLFW 3 REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIRS})

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_search_module(GLEW REQUIRED glew)
else (PKG_CONFIG_FOUND)
  message(STATUS "Could not find pkg-config, falling back on FindGLEW.cmake (untested)")
  find_package(GLEW REQUIRED)
endif (PKG_CONFIG_FOUND)
include_directories(${GLEW_INCLUDE_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})

add_executable(
  projected_area
  ${GUI_TYPE}
  src/projected_area.cpp
  src/mesh.cpp
  src/gl_error.cpp
  )

target_link_libraries(
  projected_area
  ${OpenGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${EXTRA_LIBS}
  ${FLANN_LIBRARIES}
)
