#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros-qualisys::ros-qualisys" for configuration ""
set_property(TARGET ros-qualisys::ros-qualisys APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ros-qualisys::ros-qualisys PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libros-qualisys.so.1.0.0"
  IMPORTED_SONAME_NOCONFIG "libros-qualisys.so.1.0.0"
  )

list(APPEND _cmake_import_check_targets ros-qualisys::ros-qualisys )
list(APPEND _cmake_import_check_files_for_ros-qualisys::ros-qualisys "${_IMPORT_PREFIX}/lib/libros-qualisys.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
