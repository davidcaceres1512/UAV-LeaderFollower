#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from pathlib import Path 
import xacro

# Simulation event handling 
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# Path Variables 
pkg_path = get_package_share_directory("my_uavs") # ~/colcon_ws/install/my_uavs/share/my_uavs
simulation_world_file_path = Path(pkg_path, "worlds/my_custom_world.sdf").as_posix()
# rviz_config_file_path = Path(pkg_path, "rviz/one_drone.rviz").as_posix()
# robot_description_path = Path( pkg_path, "urdf/r1.xacro")
# robot_description = {"robot_description": xacro.process_file(robot_description_path).toxml()}

simulation_cmd = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
)

# open_rviz = Node(
#         package="rviz2",
#         executable="rviz2",
#         arguments=['-d', rviz_config_file_path],
#         output="screen"
# )

# robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
# )

def generate_launch_description():
    return LaunchDescription([
        simulation_cmd,
        
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/r1/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/r1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ],
            remappings=[
                ("/r1/gazebo/command/twist", "/r1/cmd_vel"),
                ("/model/r1/odometry", "/r1/odom")
            ],
            output="screen"
        ),

        # open_rviz,

        # robot_state_publisher_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=simulation_cmd,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
