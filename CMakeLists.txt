cmake_minimum_required(VERSION 2.8.3)
project(llh_converter)

find_package(catkin REQUIRED COMPONENTS)

find_package(PkgConfig)
find_path(GeographicLib INCLUDE_DIR GeographicLib/config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_LIBRARIES
  NAMES Geographic
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES llh_converter
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

add_library(llh_converter SHARED
  src/height_converter.cpp
  src/gsigeo.cpp
  src/llh_converter.cpp
)
target_link_libraries(llh_converter PUBLIC ${catkin_LIBRARIES} Geographic)

add_executable(test test/height_converter_test.cpp)
target_link_libraries(test ${catkin_LIBRARIES} llh_converter)
add_dependencies(test ${catkin_EXPORTED_TARGETS})
