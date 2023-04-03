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
  src/meridian_convergence_angle_correction.cpp
)
target_link_libraries(llh_converter PUBLIC ${catkin_LIBRARIES} Geographic)

add_executable(llh_test test/height_converter_test.cpp)
target_link_libraries(llh_test ${catkin_LIBRARIES} llh_converter)
add_dependencies(llh_test ${catkin_EXPORTED_TARGETS})

add_executable(mcac_test test/meridian_convergence_angle_correction_test.cpp)
target_link_libraries(mcac_test ${catkin_LIBRARIES} llh_converter)
add_dependencies(mcac_test ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/llh_converter/
  DESTINATION include/llh_converter
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  TARGETS
    llh_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
