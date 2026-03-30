cmake_minimum_required(VERSION 3.0.2)
project(data_listener2)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_generation
  geometry_msgs
)

find_package(tf QUIET)
find_package(tf2_sensor_msgs QUIET)
find_package(OpenCV QUIET)
find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL QUIET)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()



catkin_install_python(PROGRAMS scripts/test_pcl.py DESTINATION lib/${PROJECT_NAME})
catkin_install_python(PROGRAMS scripts/test_pose2d.py DESTINATION lib/${PROJECT_NAME})
catkin_install_python(PROGRAMS scripts/cloud_pointcloud_web_viewer.py DESTINATION lib/${PROJECT_NAME})


add_message_files(
  FILES
  multi_lidar.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  update_gps.srv
  sync_gps.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs

)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cloud_recv_ros2_bridge
  CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/local/include"
)

add_executable(fastlio_http_sender src/fastlio_http_sender.cpp)
add_dependencies(fastlio_http_sender ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_http_sender
  ${catkin_LIBRARIES}
)

if(tf_FOUND AND tf2_sensor_msgs_FOUND AND OpenCV_FOUND AND PCL_FOUND)
  add_library(cloud_recv_ros2_bridge
    src/cloud_recv_ros2_bridge.cpp
    src/Scancontext.cpp
  )
  add_dependencies(cloud_recv_ros2_bridge ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cloud_recv_ros2_bridge
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  add_executable(cloud_recv src/cloud_recv.cpp)
  add_dependencies(cloud_recv ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cloud_recv
    cloud_recv_ros2_bridge
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  add_executable(data_sender src/data_sender.cpp src/Scancontext.cpp)
  add_dependencies(data_sender ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(data_sender
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PYTHON_LIBRARIES} ${OpenCV_LIBRARIES} gtsam
  )

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_cloud_recv_ros2_bridge
      test/test_cloud_recv_ros2_bridge.cpp
    )
    if(TARGET test_cloud_recv_ros2_bridge)
      target_link_libraries(test_cloud_recv_ros2_bridge
        cloud_recv_ros2_bridge
        gtest_main
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
      )
    endif()
  endif()
else()
  message(WARNING "Skipping cloud_recv/data_sender targets because optional full-stack dependencies are missing.")
endif()

# add_executable(test_pcl src/testpcl.cpp)
# target_link_libraries(test_pcl
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
