cmake_minimum_required(VERSION 3.1)
project(urquhart)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
)

find_package(Eigen3 REQUIRED)

SET(qhullDir "/usr/local/")
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhull_r)
LINK_DIRECTORIES(${qhullDir}/lib)
SET(qhullLibs qhullcpp qhull_r)

INCLUDE_DIRECTORIES(include include/urquhart)

catkin_package(
   INCLUDE_DIRS
   include
   LIBRARIES
   ${PROJECT_NAME}
   CATKIN_DEPENDS
   pcl_ros
   pcl_conversions
   roscpp
   DEPENDS)

add_executable(UrquhartNode
    src/test.cpp
    src/observation.cpp  
    src/utils.cpp  
    src/descriptor.cpp  
    )

target_include_directories(UrquhartNode PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(UrquhartNode PUBLIC ${catkin_LIBRARIES} ${qhullLibs})
SET_TARGET_PROPERTIES(UrquhartNode PROPERTIES
 COMPILE_DEFINITIONS "qh_QHpointer")