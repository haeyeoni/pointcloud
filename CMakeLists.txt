cmake_minimum_required(VERSION 2.8.3)
project(lidar_icp)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

        #add_include_directories("/opt/ros/melodic/include")

LIST( APPEND CMAKE_MODULE_PATH ~/g2o/cmake_modules )
find_package(G2O)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
include_directories(
        "/usr/include/eigen3"
        "/usr/include/vtk-6.3"
        "/opt/ros/melodic/include"
        src
        include
        ${PCL_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)
set(G2O_LIBS
        g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d
        )
add_executable(lidar_icp #find_nearest.cpp find_nearest.h feature_registeration.cpp feature_registeration.h visualize_correspondences.cpp visualize_correspondences.h
        nearest_callback.cpp nearest_callback.h)
target_link_libraries(lidar_icp
        ${PCL_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${G2O_LIBS}
        )
