# Install script for directory: /Users/czimbermark/Documents/TDK/pointclouds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SkeletonFinder/A_star.h;/usr/local/include/SkeletonFinder/backward.hpp;/usr/local/include/SkeletonFinder/data_type_3D.h;/usr/local/include/SkeletonFinder/skeleton_finder_3D.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/SkeletonFinder" TYPE FILE FILES
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/A_star.h"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/backward.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/data_type_3D.h"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/skeleton_finder_3D.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SkeletonFinder/quickhull/ConvexHull.hpp;/usr/local/include/SkeletonFinder/quickhull/HalfEdgeMesh.hpp;/usr/local/include/SkeletonFinder/quickhull/MathUtils.hpp;/usr/local/include/SkeletonFinder/quickhull/QuickHull.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/SkeletonFinder/quickhull" TYPE FILE FILES
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/ConvexHull.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/HalfEdgeMesh.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/MathUtils.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/QuickHull.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SkeletonFinder/quickhull/Structs/Mesh.hpp;/usr/local/include/SkeletonFinder/quickhull/Structs/Plane.hpp;/usr/local/include/SkeletonFinder/quickhull/Structs/Pool.hpp;/usr/local/include/SkeletonFinder/quickhull/Structs/Ray.hpp;/usr/local/include/SkeletonFinder/quickhull/Structs/Vector3.hpp;/usr/local/include/SkeletonFinder/quickhull/Structs/VertexDataSource.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/SkeletonFinder/quickhull/Structs" TYPE FILE FILES
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/Mesh.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/Plane.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/Pool.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/Ray.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/Vector3.hpp"
    "/Users/czimbermark/Documents/TDK/pointclouds/include/SkeletonFinder/quickhull/Structs/VertexDataSource.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libSkeletonFinder.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/Users/czimbermark/Documents/TDK/pointclouds/build/libSkeletonFinder.a")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libSkeletonFinder.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libSkeletonFinder.a")
    execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}/usr/local/lib/libSkeletonFinder.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets.cmake"
         "/Users/czimbermark/Documents/TDK/pointclouds/build/CMakeFiles/Export/9312e7228d915a311d01fc498341d50a/SkeletonFinderTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/lib/cmake/SkeletonFinder" TYPE FILE FILES "/Users/czimbermark/Documents/TDK/pointclouds/build/CMakeFiles/Export/9312e7228d915a311d01fc498341d50a/SkeletonFinderTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderTargets-release.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
      message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
      message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    file(INSTALL DESTINATION "/usr/local/lib/cmake/SkeletonFinder" TYPE FILE FILES "/Users/czimbermark/Documents/TDK/pointclouds/build/CMakeFiles/Export/9312e7228d915a311d01fc498341d50a/SkeletonFinderTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderConfigVersion.cmake;/usr/local/lib/cmake/SkeletonFinder/SkeletonFinderConfig.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/lib/cmake/SkeletonFinder" TYPE FILE FILES
    "/Users/czimbermark/Documents/TDK/pointclouds/build/SkeletonFinderConfigVersion.cmake"
    "/Users/czimbermark/Documents/TDK/pointclouds/build/SkeletonFinderConfig.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
  file(WRITE "/Users/czimbermark/Documents/TDK/pointclouds/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
