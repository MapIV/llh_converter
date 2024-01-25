cmake_minimum_required(VERSION 3.8)
project(llh_converter)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find GeographicLib
find_library(GeographicLib_LIBRARY NAMES Geographic)

# Define the library
add_library(llh_converter SHARED
  src/height_converter.cpp
  src/gsigeo.cpp
  src/llh_converter.cpp
  src/meridian_convergence_angle_correction.cpp
)
# Include directories and link libraries
target_include_directories(llh_converter PRIVATE include)
target_link_libraries(llh_converter PUBLIC ${GeographicLib_LIBRARY})

# Alias for the library target
add_library(llh_converter::llh_converter ALIAS llh_converter)
# Export the cmake config files for the build tree (this allows other projects to use this project without installing it)
export(TARGETS llh_converter NAMESPACE llh_converter:: FILE llh_converterTargets.cmake)

# Installation for the headers
install(DIRECTORY include/llh_converter/
  DESTINATION include/llh_converter
  FILES_MATCHING PATTERN "*.hpp"
)

# Installation rules for the library
install(TARGETS llh_converter
  EXPORT llh_converterTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installation rules for the cmake config files
install(EXPORT llh_converterTargets
  FILE llh_converterTargets.cmake
  DESTINATION lib/cmake/llh_converter
)

# Option to build Python bindings (default: OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings for the llh_converter library" OFF)

# Include Python bindings if the option is turned on
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
