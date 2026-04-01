# FindMyPackage.cmake
find_path(MYPACKAGE_INCLUDE_DIR NAMES mypackage.h
          PATHS /usr/include /usr/local/include)
find_library(MYPACKAGE_LIBRARY NAMES mypackage
             PATHS /usr/lib /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MyPackage DEFAULT_MSG
                                  MYPACKAGE_INCLUDE_DIR MYPACKAGE_LIBRARY)

if(MYPACKAGE_FOUND)
  set(MYPACKAGE_INCLUDE_DIRS ${MYPACKAGE_INCLUDE_DIR})
  set(MYPACKAGE_LIBRARIES ${MYPACKAGE_LIBRARY})
  # Create an imported target for modern CMake usage
  if(NOT TARGET MyPackage::MyPackage)
    add_library(MyPackage::MyPackage UNKNOWN IMPORTED)
    set_target_properties(MyPackage::MyPackage PROPERTIES
      IMPORTED_LOCATION "${MYPACKAGE_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MYPACKAGE_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(MYPACKAGE_INCLUDE_DIR MYPACKAGE_LIBRARY)
