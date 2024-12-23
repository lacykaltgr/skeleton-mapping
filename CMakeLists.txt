cmake_minimum_required(VERSION 3.5)
project(SkeletonFinder)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
#set(Torch_DIR "/app/graph/3D_Sparse_Skeleton/libtorch/share/cmake/Torch")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 -march=native ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

# Define options for building components
option(BUILD_TOOLS "Build tools" OFF)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_EXPERIMENTS "Build experiments" OFF)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Torch REQUIRED)
#find_package(CUDA REQUIRED)
find_package(Boost REQUIRED)

include_directories(
    SYSTEM 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${TORCH_LIBRARIES}
    #${CUDA_LIBRARIES}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

############################
# TESTS
############################
if(BUILD_TESTS)
    # Skeleton finder 3D test
    add_executable(skeleton_finder_3D_test
        tests/skeleton_finder_3D_test.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
    )
    target_link_libraries(skeleton_finder_3D_test
        ${PCL_LIBRARIES}
        yaml-cpp
    )

    # Skeleton finder 3D post processing test
    add_executable(skeleton_finder_3D_proc
        tests/skeleton_finder_3D_proc.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp
    )

    target_link_libraries(skeleton_finder_3D_proc
        ${PCL_LIBRARIES}
        yaml-cpp
    )
endif()

############################
# TOOLS
############################
if(BUILD_TOOLS)
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
    target_link_libraries(dense_recon
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
    )

    # MAP merger
    add_executable(map_merger
        utils/map_merger.cpp
    )
    target_link_libraries(map_merger
        ${PCL_LIBRARIES}
    )

    # downsample
    add_executable(downsample
        utils/downsample.cpp
    )
    target_link_libraries(downsample
        ${PCL_LIBRARIES}
    )

    # filter
    add_executable(filter
        utils/filter.cpp
    )
    target_link_libraries(filter
        ${PCL_LIBRARIES}
        pcl_filters
    )

    # cut_pcd
    add_executable(z_thresh
        utils/cut_pcd.cpp
    )
    target_link_libraries(z_thresh
        ${PCL_LIBRARIES}
    )

    # cut_bbx
    add_executable(cut_bbx
        utils/cut_bbx.cpp
    )
    target_link_libraries(cut_bbx
        ${PCL_LIBRARIES}
    )

    # flip_z
    add_executable(flip_z
        utils/flip_z.cpp
    )
    target_link_libraries(flip_z
        ${PCL_LIBRARIES}
    )
endif()

############################
# LIBRARY
############################

add_library(SkeletonFinder
    src/skeleton_finder_3D.cpp
    src/io.cpp
    src/pre_proc.cpp
    src/post_proc.cpp
    src/path_finder.cpp
    src/graph.cpp
    utils/quickhull/QuickHull.cpp
    utils/A_star/A_star.cpp
)

target_link_libraries(SkeletonFinder
    ${PCL_LIBRARIES}
    yaml-cpp
    Eigen3
)

############################
# EXPERIMENTS
############################
if(BUILD_EXPERIMENTS)
    add_executable(test_ekhoe
        experiments/test_ekhoe_experiment.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp

        experiments/objects.cpp
        experiments/experiments.cpp
    )
    target_link_libraries(test_ekhoe
        ${PCL_LIBRARIES}
        yaml-cpp
        nlohmann_json::nlohmann_json
    )

    add_executable(test_grid
        experiments/test_grid_experiment.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp

        experiments/experiments.cpp
        experiments/baselines.cpp
        experiments/objects.cpp
    )
    target_link_libraries(test_grid
        ${PCL_LIBRARIES}
        yaml-cpp
        nlohmann_json::nlohmann_json
    )

    add_executable(test_prm
        experiments/test_prm_experiment.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp

        experiments/experiments.cpp
        experiments/baselines.cpp
        experiments/objects.cpp
    )
    target_link_libraries(test_prm
        ${PCL_LIBRARIES}
        yaml-cpp
        nlohmann_json::nlohmann_json
    )

    add_executable(test_skeletonfinder
        experiments/test_skeletonfinder_experiment.cpp
        src/skeleton_finder_3D.cpp
        src/io.cpp
        src/pre_proc.cpp
        src/post_proc.cpp
        src/path_finder.cpp
        src/graph.cpp
        utils/quickhull/QuickHull.cpp
        utils/A_star/A_star.cpp

        experiments/experiments.cpp
        experiments/baselines.cpp
        experiments/objects.cpp
    )
    target_link_libraries(test_skeletonfinder
        ${PCL_LIBRARIES}
        yaml-cpp
        nlohmann_json::nlohmann_json
    )
endif()

######################################################################
# INSTALLATION
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
