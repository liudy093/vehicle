cmake_minimum_required(VERSION 2.8.3)
project(fast_lio2)

SET(CMAKE_BUILD_TYPE "Debug")


ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

set(FAST_LIO_CATKIN_COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
)

find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  list(APPEND FAST_LIO_CATKIN_COMPONENTS livox_ros_driver2)
  add_definitions(-DFAST_LIO_HAS_LIVOX=1)
  message(STATUS "FAST_LIO: Livox CustomMsg support enabled")
else()
  add_definitions(-DFAST_LIO_HAS_LIVOX=0)
  message(WARNING "FAST_LIO: livox_ros_driver2 not found, AVIA/CustomMsg support disabled")
endif()

find_package(catkin REQUIRED COMPONENTS ${FAST_LIO_CATKIN_COMPONENTS})

set(nabo_DIR "/usr/local/share/libnabo/cmake")
# 新增：查找 OpenCV（关键）
find_package(OpenCV REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PCL REQUIRED io common)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  /usr/include/opencv4 
  include)

add_message_files(
  FILES
  Pose6D.msg
  IrisDescriptor.msg
  LoopClosure.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(loop_closure src/loop.cpp src/Scancontext.cpp)
target_link_libraries(loop_closure ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
target_include_directories(loop_closure PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(pcl_dynamic_compress_node src/pcl_dynamic_compress_node.cpp)
target_link_libraries(pcl_dynamic_compress_node ${catkin_LIBRARIES} pcl_io pcl_common)
