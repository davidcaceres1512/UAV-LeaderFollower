#!/usr/bin/env python3

"""
@description:
This node executes the controller to solve the trajectory tracking problem using one simulated drone.
The k gain used by the controller is a ros-param to modify it at runtime.
Also, both the desired drone trajectory and the drone trajectory are published on "tr_path" and "path" 
topics, respectively, for visualizing using Rviz.
Transformation between "odom" and "base_footprint" frames is published to inform its relationship.
In the "velocity_controller" function both the desired trajectory and the signals control are computed.
For this example, the desired trajectory is a lemniscate.
Both the translational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published on the "cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "odom" topic.

-- To land the drone and finish this node, please press 'Esc' key. Do not use ctrl - C to finish this node

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs ign_world_launch.py
	# Launchs one simulated quadrotor in Ignition and runs ros_ign_bridge

$ ros2 run my_uavs quadrotor_controller_paths_urdf.py --ros-args -r __ns:=/r1
	# Runs this node. NOTE: Do not run this node within a launch file

$ rviz2 -d ~/colcon_ws/src/my_uavs/rviz/one_drone.rviz
  # Open Rviz with my configuration file to visualize paths (optional, included in the launch file)
  # Important: To visualize the path in Rviz, for a python node, set Pose Style to either arrow or axes
  # and set Buffer Length to 100

To show the node ros-params$ ros2 param dump /r1/quadrotor_controller_k_tf
To get k_gain param value$ ros2 param get /r1/quadrotor_controller_k_tf k_gain
To set k_gain param value$ ros2 param set /r1/quadrotor_controller_k_tf k_gain 0.2

To see the transformation between two frames
$ ros2 run tf2_ros tf2_echo r1/odom r1/base_footprint
"""

__author__ = "David Caceres"
__contact__ = "davidxcaceres1@gmail.com"

import rclpy, os, select, sys, math
from math import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

#To work with Euler angles and quaternions conversions
import tf_transformations

"""
It is required to install:
$ sudo apt install ros-$ROS_DISTRO-tf-transformations
$ sudo apt install python3-pip
$ sudo pip3 install transforms3d #Python package
"""

takeoff_alt = 1.2; #Desired takeoff altitude

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

key = 0

Vxy_max = 1.5; Vz_max = 0.5; Wz_max = 4 #maximum translational and rotational velocities, in [m/s] and [rad/s]
T = 100.0 #Trajectory period
k = 0.2 #controller gains kx = ky = k
ex=0.0; ey=0.0; ez=0.0; e_yaw=0.0 #Tracking errors


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TrajControl(Node):

    def __init__(self):
        super().__init__('quadrotor_controller_paths_urdf') # class’s constructor

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 2)

        self.timer = self.create_timer(0.02, self.loop) # 20ms = 50 Hz

        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback, 2)

        self.vel_msg = Twist()
        self.freq_div = 0

        self.mode = 0 #mode = 0 taking off, 1 moving around, 2 landing
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.settings = None
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        

        self.declare_parameter('k_gain', 0.4)
        global k
        k = self.get_parameter('k_gain').get_parameter_value().double_value
        self.get_logger().info('k_gain = k: %.2f' % k)


        self.frame_id = '/odom'
        self.child_frame_id = '/base_footprint'
        ns = self.get_namespace()

        if len(ns)>1:
            self.get_logger().info('The provided ns: %s' % ns)
            self.frame_id = ns+self.frame_id
            self.child_frame_id = ns+self.child_frame_id
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers to publish paths
        self.pub_path = self.create_publisher(Path, 'path', 1) #robot path
        self.pub_des_path = self.create_publisher(Path, 'des_path', 1) #desired path

        self.path_msg = Path()
        self.des_path_msg = Path()

        self.robot_points = PoseStamped()
        self.des_points = PoseStamped()
    
        self.get_logger().info('Press "ESC" key to land the drone and to finish the node')
        self.get_logger().info('Press "c" key to clean the desired trajectory')


    def __del__(self):
        self.get_logger().info('Node has been terminated')
        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        #GET THE EULER ANGLES FROM QUATERNION
        q = [0.0,0.0,0.0,1.0] #Define a unitary quaternion. Important: use [] not ()

        q[0] = msg.pose.pose.orientation.x
        q[1] = msg.pose.pose.orientation.y
        q[2] = msg.pose.pose.orientation.z
        q[3] = msg.pose.pose.orientation.w

        r, p, self.yaw = tf_transformations.euler_from_quaternion(q)


        # Create and fill out the transform broadcaster
        tranStamp = TransformStamped()

        tranStamp.header.stamp = self.get_clock().now().to_msg()
        tranStamp.header.frame_id = self.frame_id
        tranStamp.child_frame_id = self.child_frame_id

        tranStamp.transform.translation.x = self.x
        tranStamp.transform.translation.y = self.y
        tranStamp.transform.translation.z = self.z

        tranStamp.transform.rotation.x = q[0]
        tranStamp.transform.rotation.y = q[1]
        tranStamp.transform.rotation.z = q[2]
        tranStamp.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tranStamp) # Send the transformation


        # Paths
        self.path_msg.header.frame_id = self.frame_id
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info('Robot path frame_id: %s' % self.path_msg.header.frame_id, once=True)
        
        self.robot_points.header.frame_id = self.frame_id
        self.robot_points.pose.position.x = self.x
        self.robot_points.pose.position.y = self.y
        self.robot_points.pose.position.z = self.z
        self.path_msg.poses.append(self.robot_points) #Add the new point at the end of the list


    def movement(self,x,y,z,turn):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.vel_msg.linear.z = z
        self.vel_msg.angular.z = turn


    def velocity_controller(self): #Function to generate the desired trajectory and to compute the signals control
        global T, k, ex, ey, ez, e_yaw #Indicate that some variables are global
        #Desired trajectory: Lemniscate
        a = 3.0; b = 1.5; X0 = 0.0; Y0 = -0.5; Z0 = 1.5; w = 2*pi/T; c = 0.5; d = pi/2

        #Compute the time since this node started
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0

        #Desired position in the 3D space
        Xd = X0+a*math.sin(w*t)
        Yd = Y0+b*math.sin(2*w*t)
        Zd = Z0+c*math.sin(w*t) #Note: 1 <= Zd <= 2
        Yawd = d*math.sin(w*t)

        #Corresponding time derivatives
        Xdp = a*w*math.cos(w*t)
        Ydp = 2*b*w*math.cos(2*w*t)
        Zdp = c*w*math.cos(w*t)
        Yawdp = d*w*math.cos(w*t)

        #Compute tracking errors
        ex = self.x-Xd; ey = self.y-Yd; ez = self.z-Zd; e_yaw = self.yaw - Yawd; 

        #Kinematic controller. Auxiliary controls, in global coordinates
        Ux = Xdp-k*ex
        Uy = Ydp-k*ey
  
        #Translational velocities with respect to the robot frame (rotation around the Z axis)
        Vx = Ux*math.cos(self.yaw)+Uy*math.sin(self.yaw)
        Vy = -Ux*math.sin(self.yaw)+Uy*math.cos(self.yaw)

        #Kinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
        Vz = Zdp-k*ez
        Wz = Yawdp-k*e_yaw

        #Velocities saturation
        if abs(Vx)>Vxy_max:
            Vx = Vxy_max*abs(Vx)/Vx
        if abs(Vy)>Vxy_max:
            Vy = Vxy_max*abs(Vy)/Vy
        if abs(Vz)>Vz_max:
            Vz = Vz_max*abs(Vz)/Vz
        if abs(Wz)>Wz_max:
            Wz = Wz_max*abs(Wz)/Wz

        self.movement(Vx,Vy,Vz,Wz)

        # Paths
        # Desired path
        self.des_path_msg.header.frame_id = self.path_msg.header.frame_id 
        self.des_points.pose.position.x = Xd
        self.des_points.pose.position.y = Yd
        self.des_points.pose.position.z = Zd

        self.des_path_msg.poses.append(self.des_points)

        # Publish points on topics
        self.pub_path.publish(self.path_msg)
        self.pub_des_path.publish(self.des_path_msg) 


    def loop(self):
        if self.mode == 0:
            self.get_logger().info('Desired takeoff altitude = %.2f\n'
                            ' Taking off ...' % takeoff_alt, once=True)
            if self.z >= takeoff_alt:
                self.mode = 1
                self.t0 = self.get_clock().now().nanoseconds / 1e9 #Get the initial time for tracking
            else:
                self.movement(0.0,0.0,0.2,0.0)
        
        if self.mode == 1: #trajectory tracking
            self.get_logger().info('Trajectory tracking ...', once=True)
            self.velocity_controller()
        
        if self.mode == 2:
            self.get_logger().info('Landing ...', once=True)
            self.movement(0.0,0.0,-0.2,0.0)

            if self.z < 0.2: #Land
                self.destroy_node()
                sys.exit(0) #Required to release the terminal

        #Display the tracking errors
        if self.freq_div >= 25 and self.mode == 1:
            # self.get_logger().info('ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f' %(ex,ey,ez,e_yaw))
            self.freq_div = 0

            # Update k_gain param
            k = self.get_parameter('k_gain').get_parameter_value().double_value
            if k <= 0.0:
                self.get_logger().warn('k_gain cannot be 0 or lower. k = 0.1')
                k = 0.1
            if k > 2:
                self.get_logger().warn('k_gain cannot be very big. k = 2.0')
                k = 2.0
        else:
            self.freq_div = self.freq_div + 1
        
        #Check any key press
        key = get_key(self.settings)
        if key ==  '\x1b' or key == 'q' or key == 'Q': #ESC key = '\x1b'
            self.mode = 2
        if key == '\x03': #Ctrl - C
            print("Ctrl-C detected, signal_handler(signum=2)")
            self.destroy_node()
            sys.exit(0)
        if key == 'c':
            key = 0
            # self.path_msg.poses.clear()
            # del self.path_msg.poses[:]
            self.get_logger().info('Desired path reset')

        self.publisher.publish(self.vel_msg) #Publish the 4 control signals
  

def main(args=None):    
    rclpy.init(args=args)
    node = TrajControl()
    
    rclpy.spin(node) #Terminal blocked here


if __name__ == '__main__':
    main()

