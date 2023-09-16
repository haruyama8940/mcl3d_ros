cmake_minimum_required(VERSION 3.8)
project(mcl3d_ros)

# Find packages in ROS 2
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Find PCL in ROS 2
find_package(PCL REQUIRED)

# Find OpenMP (if needed)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find and include yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
endif()

# Create ROS 2 package
# ament_package()

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  "include"
)

# Add ROS 2 executables
ament_auto_add_executable(mcl 
  src/mcl_node.cpp 
  src/mcl.cpp 
  src/distance_field.cpp 
  src/imu.cpp)
target_link_libraries(mcl ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})
# ament_target_dependencies(mcl
# geometry_msgs
# nav_msgs
# )

ament_auto_add_executable(pc_to_df 
  src/pc_to_df.cpp 
  src/distance_field.cpp)
target_link_libraries(pc_to_df ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  src
)

# add_executable(sensor_points_merger src/sensor_points_merger.cpp)
# target_link_libraries(sensor_points_merger ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})
