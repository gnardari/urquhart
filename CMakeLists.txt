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
# find_package(Qhull PATHS "usr/local/include" REQUIRED NO_DEFAULT_PATH)
SET(qhullDir "/home/gnardari/thirdparty/qhull-2020.2")
INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/src)
LINK_DIRECTORIES(${qhullDir}/build)

INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(include include/urquhart)
SET(qhullLibs qhullcpp qhull_r)

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
    src/observation.cpp  
    src/utils.cpp  
    src/descriptor.cpp  
    )

target_include_directories(UrquhartNode PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(UrquhartNode PUBLIC ${catkin_LIBRARIES})
