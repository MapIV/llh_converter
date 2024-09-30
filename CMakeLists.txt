cmake_minimum_required(VERSION 3.14)
project(llh_converter)

find_package(ament_cmake_auto REQUIRED)

find_package(GeographicLib REQUIRED)

ament_auto_find_build_dependencies()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

ament_auto_add_library(llh_converter STATIC
  src/height_converter.cpp
  src/gsigeo.cpp
  src/llh_converter.cpp
  src/meridian_convergence_angle_correction.cpp
)

target_link_libraries(llh_converter PUBLIC ${GeographicLib_LIBRARY})

ament_export_include_directories("include/llh_converter")
ament_export_dependencies(Eigen3)

ament_auto_package()
