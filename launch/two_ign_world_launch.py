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
simulation_world_file_path = Path(pkg_path, "worlds/two_quadrotors_world.sdf").as_posix()
rviz_config_file_path = Path(pkg_path, "rviz/two_drones.rviz").as_posix()

ns1 = 'r1'
ns2 = 'r2'
robot_description_path1 =  Path( pkg_path, "urdf/"+ns1+".xacro")
robot_description_path2 =  Path( pkg_path, "urdf/"+ns2+".xacro")
robot_description1 = {"robot_description": xacro.process_file(robot_description_path1).toxml()}
robot_description2 = {"robot_description": xacro.process_file(robot_description_path2).toxml()}

simulation_cmd = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
)

open_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_file_path],
        output="screen"
)

robot_state_publisher_node1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns1, #It is important to use namespace when launching multiple robots
        output="both",
        parameters=[robot_description1],
)

robot_state_publisher_node2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns2,
        output="both",
        parameters=[robot_description2],
)

static_transf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transf_'+ns1+'_'+ns2,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', ns1+'/odom', '--child-frame-id', ns2+'/odom'
        ]
)

def generate_launch_description():
    return LaunchDescription([
        simulation_cmd,
        
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/"+ns1+"/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/"+ns1+"/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/"+ns2+"/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/"+ns2+"/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ],
            remappings=[
                ("/"+ns1+"/gazebo/command/twist", "/"+ns1+"/cmd_vel"),
                ("/model/"+ns1+"/odometry", "/"+ns1+"/odom"),
                ("/"+ns2+"/gazebo/command/twist", "/"+ns2+"/cmd_vel"),
                ("/model/"+ns2+"/odometry", "/"+ns2+"/odom")
            ],
            output="screen"
        ),

        open_rviz,

        robot_state_publisher_node1,
        robot_state_publisher_node2,

        static_transf_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=simulation_cmd,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
