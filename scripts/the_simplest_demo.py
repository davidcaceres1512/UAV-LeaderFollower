#!/usr/bin/env python3

"""
@description:
This node publishes velocities (in open-loop) to teleoperate a the simulated drone, and it subscribes to the drone 
odometry in order to obtain its position and its orientation.
Both the translational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published on the "/r1/cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "/r1/odom" topic, and they are displayed.

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs ign_world_launch.py
	# Launchs one simulated quadrotor in Ignition and runs ros_ign_bridge

$ ros2 run my_uavs the_simplest_demo.py
	# Runs this node (python script)
    # To give execution permission$ chmod +x the_simplest_demo.py

-- To finish this node, please press '[Ctrl]+C'. Important: drone will keep moving
"""

__author__ = "David Caceres"
__contact__ = "davidxcaceres1@gmail.com"

import rclpy#, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#To work with Euler angles and quaternions conversions
import tf_transformations

"""
It is required to install:
$ sudo apt install ros-$ROS_DISTRO-tf-transformations
$ sudo apt install python3-pip
$ sudo pip3 install transforms3d #Python package
"""

RAD2DEG = 57.295779513


class SimpleMovement(Node):

    def __init__(self):
        super().__init__('the_simplest_demo') # class’s constructor

        self.publisher = self.create_publisher(Twist, '/r1/cmd_vel', 2)

        self.timer = self.create_timer(0.05, self.loop) # 50ms = 20 Hz

        self.subscription = self.create_subscription(Odometry,'/r1/odom',self.odom_callback, 2)

        self.get_logger().warn('Node has been initialized')

        self.vel_msg = Twist()
        self.freq_div = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0



    def __del__(self):
        self.get_logger().info('Node has been terminated')
        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        #GET THE EULER ANGLES FROM QUATERNION
        q = [0.0,0.0,0.0,1.0] #Define a unitary quaternion

        q[0] = msg.pose.pose.orientation.x
        q[1] = msg.pose.pose.orientation.y
        q[2] = msg.pose.pose.orientation.z
        q[3] = msg.pose.pose.orientation.w

        r, p, y = tf_transformations.euler_from_quaternion(q)
        self.roll_deg = r * RAD2DEG
        self.pitch_deg = p * RAD2DEG
        self.yaw_deg = y * RAD2DEG


    def loop(self):
        self.vel_msg.linear.x = 0.4
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.1
        self.vel_msg.angular.z = 0.6

        # Display the drone pose using a frequency divisor
        if self.freq_div == 20:
            self.get_logger().info('x: %.2f, y: %.2f, z: %.2f, R: %.2f, P: %.2f, Y: %.2f'
                    % (self.x,self.y,self.z, self.roll_deg, self.pitch_deg, self.yaw_deg) )
            self.freq_div = 0
        else:
            self.freq_div = self.freq_div +1

        self.publisher.publish(self.vel_msg) # Publish velocity


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMovement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl-C detected, signal_handler(signum=2)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

