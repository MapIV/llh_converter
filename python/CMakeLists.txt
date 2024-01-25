cmake_minimum_required(VERSION 3.8)

# Project declaration and configuration
project(llh_converter_python
    DESCRIPTION "Python bindings for the LLH Converter library"
    LANGUAGES CXX
    VERSION 0.1.0
)

# Setting up compiler flags for C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the module name and source files
set(LLH_CONVERTER_PYTHON_MODULE_NAME llh_converter)
set(LLH_CONVERTER_PYTHON_TARGET_NAME ${LLH_CONVERTER_PYTHON_MODULE_NAME}_python)
set(LLH_CONVERTER_PYTHON_PYBIND11_PATH ${PROJECT_SOURCE_DIR}/thirdparty/pybind11)
set(LLH_CONVERTER_PYTHON_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/include)


# Define the default Python_EXECUTABLE if it is not already set
if(NOT DEFINED PYTHON_EXECUTABLE)
  find_package(Python COMPONENTS Interpreter Development)
  set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})
endif()

# Get the location of the Python site-packages directory
MESSAGE(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
execute_process(
    COMMAND
        "${PYTHON_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE
        PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Add pybind11 subdirectory
add_subdirectory(${LLH_CONVERTER_PYTHON_PYBIND11_PATH})

# Define the include directories
include_directories(
    ${LLH_CONVERTER_PYTHON_INCLUDE_PATH}
)

# Define the source files
file(GLOB_RECURSE LLH_CONVERTER_PYTHON_SOURCES ${PROJECT_SOURCE_DIR}/src/${LLH_CONVERTER_PYTHON_TARGET_NAME}/*.cpp)

# Add python module with a different name
pybind11_add_module(${LLH_CONVERTER_PYTHON_TARGET_NAME} ${LLH_CONVERTER_PYTHON_SOURCES})
# Link the library
target_link_libraries(${LLH_CONVERTER_PYTHON_TARGET_NAME} PRIVATE llh_converter::llh_converter)
# Change the output name to 'llh_converter'
set_target_properties(${LLH_CONVERTER_PYTHON_TARGET_NAME} PROPERTIES OUTPUT_NAME ${LLH_CONVERTER_PYTHON_MODULE_NAME})
# Configure install targets
install(TARGETS ${LLH_CONVERTER_PYTHON_TARGET_NAME} LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES})
