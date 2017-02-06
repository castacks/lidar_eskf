cmake_minimum_required(VERSION 2.8.3)
project(lidar_eskf)

option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  sensor_msgs
  tf
  eigen_conversions
  tf_conversions
  pcl_conversions
  pcl_ros
  laser_geometry
  dynamicEDT3D
)

find_package(Boost REQUIRED COMPONENTS system random)
find_package(Eigen REQUIRED)
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
add_library(eskf src/eskf.cpp)
target_link_libraries(eskf ${catkin_LIBRARIES})
add_library(particles src/particles.cpp)
target_link_libraries(particles ${catkin_LIBRARIES})
add_library(map src/map.cpp)
target_link_libraries(map ${catkin_LIBRARIES})
add_library(gpf src/gpf.cpp)
target_link_libraries(gpf eskf particles ${catkin_LIBRARIES})

#add_executable(eskf_test src/eskf_test.cpp)
#target_link_libraries(eskf_test eskf ${catkin_LIBRARIES})
#add_executable(gpf_test src/gpf_test.cpp)
#target_link_libraries(gpf_test eskf map gpf particles ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
add_executable(bag_to_pcd src/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd ${PCL_LIBRARY_DIRS} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(lidar_eskf_node src/lidar_eskf_node.cpp)
target_link_libraries(lidar_eskf_node eskf map gpf particles ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} )
