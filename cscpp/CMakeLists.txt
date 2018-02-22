cmake_minimum_required(VERSION 2.8.3)
project( cscpp )
SET ( CMAKE_CXX_FLAGS "-frounding-math" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  shape_msgs
  std_msgs
  rviz_visual_tools
  sspp
  )

# PCL causes issues with CUDA
#find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
# CGAL and its components
find_package(CGAL QUIET COMPONENTS)

if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()

# Few differences when dealing with ros-indigo and ros-kinetic
set( ROS_DISTRO $ENV{ROS_DISTRO} )
if( ${ROS_DISTRO} STREQUAL "indigo" )
    find_package(fcl REQUIRED)
elseif(${ROS_DISTRO} STREQUAL "kinetic")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBFCL REQUIRED fcl)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message("Unsupported ROS Distro")
endif()


#Cuda related Flags
#find_package(CUDA 8.0 REQUIRED EXACT)
find_package(CUDA REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENMP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES occlusion_culling
   CATKIN_DEPENDS roscpp pcl_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# add_library(fcl_test src/fcl_utility.cpp ) ##check if you still need it

add_library(voxel_grid_occlusion_estimation src/voxel_grid_occlusion_estimation.cpp )

add_library(frustum_culling src/frustum_culling.cpp )
target_link_libraries(frustum_culling ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES})

add_library(occlusion_culling src/occlusion_culling.cpp)
target_link_libraries(occlusion_culling frustum_culling voxel_grid_occlusion_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if (CUDA_FOUND)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    #-gencode arch=compute_61,code=sm_61
    #-gencode arch=compute_62,code=sm_62
  )
  
###libs that uses cuda
    cuda_add_library(frustum_culling_gpu src/frustum_culling_gpu.cu )
    cuda_add_library(voxel_grid_occlusion_estimation_gpu src/voxel_grid_occlusion_estimation_gpu.cu )
    add_library(occlusion_culling_gpu src/occlusion_culling_gpu.cpp)
    target_link_libraries(occlusion_culling_gpu frustum_culling_gpu voxel_grid_occlusion_estimation_gpu ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    cuda_add_executable(test_cuda src/test_cuda.cu)     

##coverage heuristic
    add_library(CoverageHeuristic SHARED src/coverage_path_planning_heuristic.cpp)
    target_link_libraries(CoverageHeuristic ${component_test_LIBRARIES} occlusion_culling_gpu ${OCTOMAP_LIBRARIES})
    target_include_directories(CoverageHeuristic PUBLIC include ${catkin_INCLUDE_DIRS} /usr/local/cuda/include/)
        
    add_executable(coverage_heuristic_test src/coverage_heuristic_test.cpp)
    target_include_directories(coverage_heuristic_test PUBLIC include ${catkin_INCLUDE_DIRS} /usr/local/cuda/include/)
    target_link_libraries(coverage_heuristic_test SSPathPlanner
        -lgmp -lmpfr
        ${CGAL_LIBRARY}
        ${CGAL_Core_LIBRARY}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${cscpp_LIBRARIES}
        occlusion_culling_gpu
        ${rviz_visual_tools_LIBRARIES}
        CoverageHeuristic
        )
        
endif (CUDA_FOUND)
