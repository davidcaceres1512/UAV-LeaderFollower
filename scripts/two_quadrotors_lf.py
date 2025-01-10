#!/usr/bin/env python3

"""
@description:
This node executes the controller in order to solve the formation control problem, using the leader-follower 
scheme, with two simulated drones. The leader drone tracks its desired trajectory (using the velocity 
controller), and then the follower drone follows the offset leader's trajectory.
For this example, a helix trajectory is used.
This node also publish transformation between robots frame.

-- To land the drone and finish this node, please press 'Esc' key. Do not use ctrl - C to finish this node

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs two_ign_world_launch.py
	# Launchs two simulated quadrotors in Ignition, runs ros_gz_bridge and opens Rviz

$ ros2 run my_uavs two_quadrotors_lf.py r1 r2
	# Runs this node. NOTE: Do not run this node within a launch file
"""

__author__ = "David Caceres"
__contact__ = "davidxcaceres1@gmail.com"

import rclpy, os, select, sys, math
from math import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
T = 50.0 #Trajectory period
k = 0.4 #controller gains kx = ky = k

#Follower drone variables
s_d = 1.5; alp_d = pi; s_zd = 0.0


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


class LFController(Node):

    def __init__(self, argv):
        super().__init__('two_quadrotors_lf') # class’s constructor

        self.ns1 = argv[1]
        self.ns2 = argv[2]

        self.publisher1 = self.create_publisher(Twist, self.ns1+'/cmd_vel', 2)
        self.publisher2 = self.create_publisher(Twist, self.ns2+'/cmd_vel', 2)

        self.timer = self.create_timer(0.02, self.loop) # 20ms = 50 Hz

        self.subscription1 = self.create_subscription(Odometry, self.ns1+'/odom',self.odom_callback1, 2)
        self.subscription2 = self.create_subscription(Odometry, self.ns2+'/odom',self.odom_callback2, 2)

        self.vel_msg1 = Twist()
        self.vel_msg2 = Twist()

        self.freq_div = 0
        self.mode = 0 #mode = 0 taking off, 1 moving around, 2 landing

        #Define the robots' posture using lists [x,y,z,roll,pitch,yaw]
        self.r_pose1 = [0.0,0.0,0.0,0.0,0.0,0.0] 
        self.r_pose2 = [0.0,0.0,0.0,0.0,0.0,0.0]

        #Signals control of the leader required by the follower
        self.Vxl = 0.0;  self.Vyl = 0.0; self.Vzl = 0.0; self.Wzl = 0.0 

        self.settings = None
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)


        self.frame_id = '/odom'
        self.child_frame_id = '/base_footprint'
        
        # Initialize the transform broadcaster
        self.tf_broadcaster1 = TransformBroadcaster(self)
        self.tf_broadcaster2 = TransformBroadcaster(self)
    
        self.get_logger().info('Press "ESC" to land the drone and to finish the node')


    def __del__(self):
        self.get_logger().info('Node has been terminated')
        

    def odom_callback1(self, msg):
        self.r_pose1[0] = msg.pose.pose.position.x
        self.r_pose1[1] = msg.pose.pose.position.y
        self.r_pose1[2] = msg.pose.pose.position.z

        #GET THE EULER ANGLES FROM QUATERNION
        q = [0.0,0.0,0.0,1.0] #Define a unitary quaternion. Important: use [] not ()

        q[0] = msg.pose.pose.orientation.x
        q[1] = msg.pose.pose.orientation.y
        q[2] = msg.pose.pose.orientation.z
        q[3] = msg.pose.pose.orientation.w

        self.r_pose1[3], self.r_pose1[4], self.r_pose1[5] = tf_transformations.euler_from_quaternion(q)


        # Create and fill out the transform broadcaster
        tranStamp = TransformStamped()

        tranStamp.header.stamp = self.get_clock().now().to_msg()
        tranStamp.header.frame_id = self.ns1+self.frame_id
        tranStamp.child_frame_id = self.ns1+self.child_frame_id

        tranStamp.transform.translation.x = self.r_pose1[0]
        tranStamp.transform.translation.y = self.r_pose1[1]
        tranStamp.transform.translation.z = self.r_pose1[2]

        tranStamp.transform.rotation.x = q[0]
        tranStamp.transform.rotation.y = q[1]
        tranStamp.transform.rotation.z = q[2]
        tranStamp.transform.rotation.w = q[3]

        self.tf_broadcaster1.sendTransform(tranStamp) # Send the transformation
    

    def odom_callback2(self, msg):
        self.r_pose2[0] = msg.pose.pose.position.x
        self.r_pose2[1] = msg.pose.pose.position.y
        self.r_pose2[2] = msg.pose.pose.position.z

        #GET THE EULER ANGLES FROM QUATERNION
        q = [0.0,0.0,0.0,1.0] #Define a unitary quaternion. Important: use [] not ()

        q[0] = msg.pose.pose.orientation.x
        q[1] = msg.pose.pose.orientation.y
        q[2] = msg.pose.pose.orientation.z
        q[3] = msg.pose.pose.orientation.w

        self.r_pose2[3], self.r_pose2[4], self.r_pose2[5] = tf_transformations.euler_from_quaternion(q)


        # Create and fill out the transform broadcaster
        tranStamp = TransformStamped()

        tranStamp.header.stamp = self.get_clock().now().to_msg()
        tranStamp.header.frame_id = self.ns2+self.frame_id
        tranStamp.child_frame_id = self.ns2+self.child_frame_id

        tranStamp.transform.translation.x = self.r_pose2[0]
        tranStamp.transform.translation.y = self.r_pose2[1]
        tranStamp.transform.translation.z = self.r_pose2[2]

        tranStamp.transform.rotation.x = q[0]
        tranStamp.transform.rotation.y = q[1]
        tranStamp.transform.rotation.z = q[2]
        tranStamp.transform.rotation.w = q[3]

        self.tf_broadcaster2.sendTransform(tranStamp) # Send the transformation


    def movement1(self,x,y,z,turn):
        self.vel_msg1.linear.x = x
        self.vel_msg1.linear.y = y
        self.vel_msg1.linear.z = z
        self.vel_msg1.angular.z = turn

    def movement2(self,x,y,z,turn):
        self.vel_msg2.linear.x = x
        self.vel_msg2.linear.y = y
        self.vel_msg2.linear.z = z
        self.vel_msg2.angular.z = turn

    
    def leader_vel_control(self): #Function to generate the desired trajectory and to compute the signals control
        global T, k, takeoff_alt #Indicate that some variables are global
        #Desired trajectory: Helix
        X0 = 0.0; Y0 = 0.0; Z0 = takeoff_alt; radius = 3.0; w = 2*pi/T; Vzd = 0.02

        #Compute the time since this node started
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0

        #Desired position in the 3D space
        Xd = X0+radius*math.sin(w*t)
        Yd = Y0+radius*math.cos(w*t)
        Zd = Z0+Vzd*t #linear increasing
        Yawd = math.sin(w*t) #sinusoidal behavior

        #Corresponding time derivatives
        Xdp = radius*w*math.cos(w*t)
        Ydp = -radius*w*math.sin(w*t)
        Zdp = Vzd
        Yawdp = w*math.cos(w*t)

        #Compute tracking errors
        ex = self.r_pose1[0]-Xd; ey = self.r_pose1[1]-Yd; ez = self.r_pose1[2]-Zd
        e_yaw = self.r_pose1[5] - Yawd; 

        #Kinematic controller. Auxiliary controls, in global coordinates
        Ux = Xdp-k*ex
        Uy = Ydp-k*ey
  
        #Translational velocities with respect to the robot frame (rotation around the Z axis)
        Vx = Ux*math.cos(self.r_pose1[5])+Uy*math.sin(self.r_pose1[5])
        Vy = -Ux*math.sin(self.r_pose1[5])+Uy*math.cos(self.r_pose1[5])

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

        self.movement1(Vx,Vy,Vz,Wz)

        self.Vxl = Vx
        self.Vyl = Vy
        self.Vzl = Vz
        self.Wzl = Wz


    def follower_vel_control(self): #Function to compute the signals control of the follower robot
        #Note: desired follower position and yaw orientation are given by the leader's ones
        global k, s_d, alp_d, s_zd        

        #Follower desired position with respect to the global frame
        Xfd = self.r_pose1[0]+s_d*math.cos(self.r_pose1[5]+alp_d)
        Yfd = self.r_pose1[1]+s_d*math.sin(self.r_pose1[5]+alp_d)

        #Translational velocities of the leader drone in the global frame
        xlp = self.Vxl*math.cos(self.r_pose1[5])-self.Vyl*math.sin(self.r_pose1[5])
        ylp = self.Vxl*math.sin(self.r_pose1[5])+self.Vyl*math.cos(self.r_pose1[5])

        #Corresponding desired time derivatives
        Xfdp = xlp-self.Wzl*s_d*math.sin(self.r_pose1[5]+alp_d)
        Yfdp = ylp+self.Wzl*s_d*math.cos(self.r_pose1[5]+alp_d)

        #Compute tracking errors (with respect to the global frame)
        exf = self.r_pose2[0]-Xfd
        eyf = self.r_pose2[1]-Yfd; 
        ezf = self.r_pose2[2]-(self.r_pose1[2]+s_zd) #Both the altitude and the orientation are equal to leader drone's ones
        e_yawf = self.r_pose2[5] - self.r_pose1[5]

        #Kinematic controller. Auxiliary controls, in global coordinates
        Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf
      
        #Translational velocities with respect to the robot frame
        Vxf = Ux*math.cos(self.r_pose2[5])+Uy*math.sin(self.r_pose2[5])
        Vyf = -Ux*math.sin(self.r_pose2[5])+Uy*math.cos(self.r_pose2[5])

        #Vertical and rotational velocities
        Vzf = self.Vzl-k*ezf
        Wzf = self.Wzl-k*e_yawf
    
        #Velocities saturation
        if abs(Vxf)>Vxy_max:
            Vxf = Vxy_max*abs(Vxf)/Vxf
        if abs(Vyf)>Vxy_max:
            Vyf = Vxy_max*abs(Vyf)/Vyf
        if abs(Vzf)>Vz_max:
            Vzf = Vz_max*abs(Vzf)/Vzf
        if abs(Wzf)>Wz_max:
            Wzf = Wz_max*abs(Wzf)/Wzf

        self.movement2(Vxf,Vyf,Vzf,Wzf)


    def loop(self):
        if self.mode == 0:
            self.get_logger().info('Desired takeoff altitude = %.2f\n'
                            ' Taking off ...' % takeoff_alt, once=True)
            if self.r_pose1[2] >= takeoff_alt or self.r_pose2[2] >= takeoff_alt:
                self.mode = 1
                self.t0 = self.get_clock().now().nanoseconds / 1e9 #Get the initial time for tracking
            else:
                self.movement1(0.0,0.0,0.2,0.0)
                self.movement2(0.0,0.0,0.2,0.0)
        
        if self.mode == 1: #trajectory tracking
            self.get_logger().info('Trajectory tracking ...', once=True)
            self.leader_vel_control()
            self.follower_vel_control()
        
        if self.mode == 2: #Land
            self.get_logger().info('Landing ...', once=True)
            self.movement1(0.0,0.0,-0.2,0.0)
            self.movement2(0.0,0.0,-0.2,0.0)

            if self.r_pose1[2] < 0.2: #Stop robot to wait for the other
                self.movement1(0.0,0.0,0.0,0.0)

            if self.r_pose2[2] < 0.2:
                self.movement2(0.0,0.0,0.0,0.0)
            
            if self.r_pose1[2] < 0.2 and self.r_pose2[2] < 0.2:
                self.destroy_node()
                sys.exit(0) #Required to release the terminal

        #Display some variables of interest
        if self.freq_div >= 25 and self.mode == 1:
            # self.get_logger().info('' %())
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

        self.publisher1.publish(self.vel_msg1) #Publish the 4 control signals
        self.publisher2.publish(self.vel_msg2)
  

def main(args=None): 
    logger = rclpy.logging.get_logger('logger')

    logger.info('argc = %d' % len(sys.argv))

    #Obtain parameters from command line arguments
    if len(sys.argv) < 3: #node_name and ns1, ns2 = 3; if this node is run within a launcher, node_name and ns1, ns2 --ros-args = 4
        logger.error('Robots namespace not given\nUsage: $ ros2 run my_uavs two_quadrotors_lf.py r1 r2')
        sys.exit(1)
  
    rclpy.init(args=args)
    node = LFController(sys.argv)
    
    rclpy.spin(node) #Terminal blocked here


if __name__ == '__main__':
    main()

