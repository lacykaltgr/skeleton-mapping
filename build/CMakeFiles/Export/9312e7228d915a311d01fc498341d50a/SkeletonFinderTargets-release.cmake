#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SkeletonFinder" for configuration "Release"
set_property(TARGET SkeletonFinder APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SkeletonFinder PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libSkeletonFinder.a"
  )

list(APPEND _cmake_import_check_targets SkeletonFinder )
list(APPEND _cmake_import_check_files_for_SkeletonFinder "/usr/local/lib/libSkeletonFinder.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
