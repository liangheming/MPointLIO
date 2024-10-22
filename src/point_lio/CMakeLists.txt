cmake_minimum_required(VERSION 3.0.2)
project(point_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  livox_ros_driver2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_lio
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(SRC_LIST src/commons.cpp
             src/eskf.cpp
             src/ikd_Tree.h
             src/ikd_Tree.cpp
             src/map_builder.cpp
             src/point_lio_ros.cpp)
             
add_executable(point_lio_node src/point_lio_node.cpp ${SRC_LIST})
target_link_libraries(point_lio_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
