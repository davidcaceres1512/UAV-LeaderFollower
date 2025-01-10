#!/usr/bin/env python3

"""
@description:
This node executes the controller to solve the trajectory tracking problem using one simulated drone.
In the "velocity_controller" function both the desired trajectory and the signals control are computed.
For this example, the desired trajectory is a lemniscate.
Both the translational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published on the "cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "odom" topic.

-- To land the drone and finish this node, please press 'Esc' key. Do not use ctrl - C to finish this node

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs ign_world_launch.py
	# Launchs one simulated quadrotor in Ignition and runs ros_ign_bridge

$ ros2 run my_uavs quadrotor_controller.py --ros-args -r __ns:=/r1
	# Runs this node. NOTE: Do not run this node within a launch file
"""

__author__ = "David Caceres"
__contact__ = "davidxcaceres1@gmail.com"

import rclpy, os, select, sys, math
from math import pi
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

takeoff_alt = 0.6; #Desired takeoff altitude


if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

key = 0

Vxy_max = 1.5; Vz_max = 0.5; Wz_max = 4 #maximum translational and rotational velocities, in [m/s] and [rad/s]
T = 100.0; k = 0.4 #Trajectory period, controller gains kx = ky = k
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
        super().__init__('quadrotor_controller') # class’s constructor

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 2)

        self.timer = self.create_timer(0.02, self.loop) # 20ms = 50 Hz

        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback, 2)

        self.get_logger().warn('Node has been initialized')
        self.get_logger().info('Press "ESC" to land the drone and to finish the node')

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
            self.get_logger().info('ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f' %(ex,ey,ez,e_yaw))
            self.freq_div = 0
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

        self.publisher.publish(self.vel_msg) #Publish the 4 control signals
  

def main(args=None):    
    rclpy.init(args=args)
    node = TrajControl()
    
    rclpy.spin(node) #Terminal blocked here


if __name__ == '__main__':
    main()

