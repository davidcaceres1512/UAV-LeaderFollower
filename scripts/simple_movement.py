#!/usr/bin/env python3

"""
@description:
This node publishes velocities (in open-loop) to teleoperate a the simulated drone, and it subscribes to the drone 
odometry in order to obtain its position and its orientation.
First, the drone takes off until reach a desired altitude.
Both the translational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published on the "/cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "/odom" topic, and they are displayed.

-- To land the drone and finish this node, please press 'Esc' key. Do not use ctrl - C to finish this node

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs ign_world_launch.py
	# Launchs one simulated quadrotor in Ignition and runs ros_ign_bridge

$ ros2 run my_uavs simple_movement.py --ros-args -r __ns:=/r1
	# Runs this node. NOTE: Do not run this node within a launch file
"""

__author__ = "David Caceres"
__contact__ = "davidxcaceres1@gmail.com"

import rclpy, os, select, sys
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
takeoff_alt = 1.2; #Desired takeoff altitude


if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

key = 0

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


class SimpleMovement(Node):

    def __init__(self):
        super().__init__('simple_movement') # class’s constructor

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 2)

        self.timer = self.create_timer(0.05, self.loop) # 50ms = 20 Hz

        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback, 2)

        self.get_logger().warn('Node has been initialized')
        self.get_logger().info('Press "ESC" to land the drone and finish the node')

        self.vel_msg = Twist()
        self.freq_div = 0

        self.mode = 0 #mode = 0 taking off, 1 moving around, 2 landing
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.settings = None
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)


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

        r, p, y = tf_transformations.euler_from_quaternion(q)
        self.roll_deg = r * RAD2DEG
        self.pitch_deg = p * RAD2DEG
        self.yaw_deg = y * RAD2DEG

        """GET THE QUATERNION FROM THE EULER ANGLES
        quat = tf_transformations.quaternion_from_euler(0,0,0)
        print("quat.x:",quat[0])
        """
        


    def movement(self,x,y,z,turn):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.vel_msg.linear.z = z
        self.vel_msg.angular.z = turn


    def loop(self):
        if self.mode == 0:
            self.get_logger().info('Desired takeoff altitude = %.2f\n'
                            ' Taking off ...' % takeoff_alt, once=True)
            if self.z >= takeoff_alt:
                self.mode = 1
            else:
                self.movement(0.0,0.0,0.2,0.0)
        
        if self.mode == 1: #Circular movement on the horizontal plane
            self.get_logger().info('Circular movement ...', once=True)
            self.movement(0.4,0.0,0.0,0.5)
        
        if self.mode == 2:
            self.get_logger().info('Landing ...', once=True)
            self.movement(0.0,0.0,-0.2,0.0)

            if self.z < 0.2:
                self.destroy_node()
                sys.exit(0) #Required to release the terminal

        #Display the drone pose using a frequency divisor
        if self.freq_div == 20:
            self.get_logger().info('x: %.2f, y: %.2f, z: %.2f, R: %.2f, P: %.2f, Y: %.2f'
                    % (self.x,self.y,self.z, self.roll_deg, self.pitch_deg, self.yaw_deg) )
            self.freq_div = 0
        else:
            self.freq_div = self.freq_div +1
        
        #Check any key press
        key = get_key(self.settings)
        if key ==  '\x1b' or key == 'q' or key == 'Q': #ESC key = '\x1b'
            self.mode = 2
        if key == '\x03': #Ctrl - C
            print("Ctrl-C detected, signal_handler(signum=2)")
            self.destroy_node()
            sys.exit(0)

        self.publisher.publish(self.vel_msg) # Publish velocity   
  

def main(args=None):    
    rclpy.init(args=args)
    node = SimpleMovement()
    
    rclpy.spin(node) #Terminal blocked here


if __name__ == '__main__':
    main()

