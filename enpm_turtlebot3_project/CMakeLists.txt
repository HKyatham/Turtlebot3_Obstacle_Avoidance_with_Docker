cmake_minimum_required(VERSION 3.8)
project(enpm_turtlebot3_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "gazebo_ros_pkgs"
)

set(EXEC_NAME "turtlebot3_drive")

add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

add_executable(lin_vel_publisher src/lin_vel_publisher.cpp)
ament_target_dependencies(lin_vel_publisher rclcpp std_msgs)

# add_executable("velocity_pub" src/velocity_publisher.cpp)
# ament_target_dependencies("velocity_pub" ${dependencies})

# add_library(${PROJECT_NAME} SHARED src/plannar_mover.cpp)
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# add_library(${PROJECT_NAME} SHARED src/waypoint_plugin.cpp)
# target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

message(STATUS "Gazebo Libraries: ${GAZEBO_LIBRARIES}")
message(STATUS "Gazebo Include Dirs: ${GAZEBO_INCLUDE_DIRS}")



install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lin_vel_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

#--------------------------------
# Python
#--------------------------------

# install Python modules from first_package/first_package
# ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# install Python executables
install(PROGRAMS scripts/control.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pid_controller.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/path_finder.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/turtlebot_teleop.py DESTINATION lib/${PROJECT_NAME})

ament_package()