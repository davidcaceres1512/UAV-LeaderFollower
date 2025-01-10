cmake_minimum_required(VERSION 3.5)
project(my_uavs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies "rclcpp" "geometry_msgs" "nav_msgs" "tf2" "tf2_ros")

add_executable(the_simplest_demo src/the_simplest_demo.cpp)
ament_target_dependencies(the_simplest_demo ${dependencies})

add_executable(simple_movement src/simple_movement.cpp)
ament_target_dependencies(simple_movement ${dependencies})

add_executable(quadrotor_controller src/quadrotor_controller.cpp)
ament_target_dependencies(quadrotor_controller ${dependencies})

add_executable(quadrotor_controller_k_tf src/quadrotor_controller_k_tf.cpp)
ament_target_dependencies(quadrotor_controller_k_tf ${dependencies})

add_executable(quadrotor_controller_paths_urdf src/quadrotor_controller_paths_urdf.cpp)
ament_target_dependencies(quadrotor_controller_paths_urdf ${dependencies})

add_executable(two_quadrotors_lf src/two_quadrotors_lf.cpp)
ament_target_dependencies(two_quadrotors_lf ${dependencies})

add_executable(two_quadrotors_paths src/two_quadrotors_paths.cpp)
ament_target_dependencies(two_quadrotors_paths ${dependencies})


install(TARGETS
  the_simplest_demo
  simple_movement
  quadrotor_controller
  quadrotor_controller_k_tf
  quadrotor_controller_paths_urdf
  two_quadrotors_lf
  two_quadrotors_paths
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch urdf rviz worlds src scripts models
  DESTINATION share/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME}) #It is required the __init__.py inside the PROJECT_NAME folder

# Install Python executables #They must start with the shebag tag and must have execution permision
install(PROGRAMS
  scripts/the_simplest_demo.py
  scripts/simple_movement.py
  scripts/quadrotor_controller.py
  scripts/quadrotor_controller_k_tf.py
  scripts/quadrotor_controller_paths_urdf.py #paths are not visualized as desired
  scripts/two_quadrotors_lf.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()


