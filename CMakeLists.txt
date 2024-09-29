cmake_minimum_required(VERSION 3.0.2)
project(SkeletonFinder)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
#set(Torch_DIR "/app/graph/3D_Sparse_Skeleton/libtorch/share/cmake/Torch")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 -march=native ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)
#find_package(Torch REQUIRED)
#find_package(CUDA REQUIRED)

include_directories(
    SYSTEM 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${TORCH_LIBRARIES}
    #${CUDA_LIBRARIES}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

# Omni2Colmap
add_executable(omni2colmap
    utils/omni2colmap.cpp
)

target_link_libraries(omni2colmap
    nlohmann_json::nlohmann_json
)

# Dense reconstruction
add_executable(dense_recon
    utils/dense_reconstruction.cpp
)

# Dense reconstruction
add_executable(ctreep_test
    utils/CTreeP_test.cpp
)

target_link_libraries(dense_recon
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# Skeleton finder 3D test
add_executable(skeleton_finder_3D_test
    src/skeleton_finder_3D_test.cpp
    src/skeleton_finder_3D.cpp
    src/io.cpp
    utils/quickhull/QuickHull.cpp
    utils/A_star/A_star.cpp
)

target_link_libraries(skeleton_finder_3D_test
    ${PCL_LIBRARIES}
    yaml-cpp
)

target_link_libraries(ctreep_test
    ${PCL_LIBRARIES}
    jsoncpp
)

#add_executable(dense_recon_parallel
#    utils/dense_reconstruction_parallel.cpp
#)

#target_link_libraries(dense_recon_parallel
#    ${PCL_LIBRARIES}
#    ${OpenCV_LIBS}
#    nlohmann_json::nlohmann_json
#    #${TORCH_LIBRARIES}
#    #${CUDA_LIBRARIES}
#)


add_library(SkeletonFinder
    src/skeleton_finder_3D_test.cpp
    src/skeleton_finder_3D.cpp
    src/io.cpp
    utils/quickhull/QuickHull.cpp
    utils/A_star/A_star.cpp
)

target_link_libraries(SkeletonFinder
    ${PCL_LIBRARIES}
    yaml-cpp
    Eigen3
)
######################################################################

target_include_directories(SkeletonFinder PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/SkeletonFinder>
)
 
# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(INCLUDES_INSTALL_DIR ${INCLUDES_DESTINATION}/SkeletonFinder)
set(CMAKECONFIG_INSTALL_DIR ${LIBRARY_DESTINATION}/cmake/SkeletonFinder)
 
# this relative path allows installed files to be relocatable. (Gabor: not sure whetherthis is needed or not)
file(RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKECONFIG_INSTALL_DIR}"
    "${INCLUDES_INSTALL_DIR}"
)
set(EXPORT_LIB_INC_DIRS "\${PROJECT_CMAKE_DIR}/${REL_INCLUDE_DIR}")
 
 
 
file(GLOB SKELETON_FINDER_HEADERS
 "include/SkeletonFinder/*.h" 
 "include/SkeletonFinder/*.hpp"
)
file(GLOB SKELETON_FINDER_QUICKHULL_HEADERS
 "include/SkeletonFinder/quickhull/*.hpp"
)
file(GLOB SKELETON_FINDER_QUICKHULL_STRUCTS_HEADERS
    "include/SkeletonFinder/quickhull/Structs/*.hpp"
)
install(FILES ${SKELETON_FINDER_HEADERS} DESTINATION ${INCLUDES_INSTALL_DIR})
install(FILES ${SKELETON_FINDER_QUICKHULL_HEADERS} DESTINATION ${INCLUDES_INSTALL_DIR}/quickhull)
install(FILES ${SKELETON_FINDER_QUICKHULL_STRUCTS_HEADERS} DESTINATION ${INCLUDES_INSTALL_DIR}/quickhull/Structs)
 
# install libraries
install(TARGETS SkeletonFinder
    EXPORT SkeletonFinderTargets
    RUNTIME DESTINATION ${RUNTIME_DESTINATION}
    LIBRARY DESTINATION ${LIBRARY_DESTINATION}
    ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
    INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

 
# write package config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/SkeletonFinderConfigVersion.cmake"
    VERSION 1.0
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/SkeletonFinderConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/SkeletonFinderConfig.cmake"
    INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)
install(EXPORT SkeletonFinderTargets
    DESTINATION ${CMAKECONFIG_INSTALL_DIR})
install(FILES "${PROJECT_BINARY_DIR}/SkeletonFinderConfigVersion.cmake"
    "${PROJECT_BINARY_DIR}/SkeletonFinderConfig.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)
