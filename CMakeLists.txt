cmake_minimum_required(VERSION 2.8.3)
project(lidar_eskf)

option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
  sensor_msgs
  tf
  eigen_conversions
  tf_conversions
  pcl_conversions
  pcl_ros
  dynamicEDT3D
  laser_geometry
)

find_package(Boost REQUIRED COMPONENTS system random)
find_package(Eigen REQUIRED)
find_package(message_generation)
find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(dynamicEDT3D REQUIRED)
find_package(OpenMP)


# add OpenMP support
if (WITH_OPENMP)
    if (OPENMP_FOUND)
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()

    if (NOT OPENMP_FOUND)
        set(WITH_OPENMP OFF)
    endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS tf
  DEPENDS octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${dynamicEDT3D_INCLUDE_DIRS}
)

## Declare C++ library
add_library(imu src/ImuOdom.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

add_executable(imu_test src/imu_test.cpp)
target_link_libraries(imu_test imu ${catkin_LIBRARIES}  )
