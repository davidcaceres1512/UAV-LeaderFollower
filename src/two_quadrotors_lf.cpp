/*
@description:
This node executes the controller in order to solve the formation control problem, using the leader-follower 
scheme, with two simulated drones. The leader drone tracks its desired trajectory (using the velocity 
controller), and then the follower drone follows the offset leader's trajectory.
For this example, a helix trajectory is used.

-- To land the drone and finish this node, please press 'Esc' key. Do not use ctrl - C to finish this node

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs two_ign_world_launch.py
	# Launchs two simulated quadrotors in Ignition, runs ros_gz_bridge and opens Rviz

$ ros2 run my_uavs two_quadrotors_lf r1 r2
	# Runs this node. NOTE: Do not run this node within a launch file
*/

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//Includes needed by keyboard events
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Create a custom structure to define the robot posture (position and orientation, using Euler angles)
struct robot_pose{ double x, y, z, roll, pitch, yaw; };

double takeoff_alt = 1.2; //Desired takeoff altitude
int key;

//Trajectory variables
double Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4; //maximum translational and rotational velocities, in [m/s] and [rad/s]
double T = 50, k; //Trajectory period, controller gains kx = ky = k

//Follower drone variables
double s_d = 1.5, alp_d = M_PI, s_zd = 0;


int kbhit(void){ //kbhit function
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

class LFController : public rclcpp::Node
{
  public:
    LFController( char * argv[] ) : Node("two_quadrotors_lf")
    {
      ns1_ = argv[1];
      ns2_ = argv[2];

      publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>(ns1_+"/cmd_vel", 2);
      publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>(ns2_+"/cmd_vel", 2);

      timer_ = this->create_wall_timer(20ms, std::bind(&LFController::loop, this)); //20ms = 50 Hz
      
      subscriber1_ = this->create_subscription<nav_msgs::msg::Odometry>(ns1_+"/odom", 
            2, std::bind(&LFController::odom_callback1, this, _1));
      
      subscriber2_ = this->create_subscription<nav_msgs::msg::Odometry>(ns2_+"/odom", 
            2, std::bind(&LFController::odom_callback2, this, _1));
      
      RCLCPP_INFO(this->get_logger(), "Press 'ESC' key to land the drones and to finish the node");
    }
    
    ~LFController() //Class destructor definition
    {
      RCLCPP_INFO(this->get_logger(), "Node has been terminated"); 
    }
    
    void odom_callback1(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      r_pose1_.x = msg->pose.pose.position.x;
      r_pose1_.y = msg->pose.pose.position.y;
      r_pose1_.z = msg->pose.pose.position.z;
      
      //GET THE EULER ANGLES FROM THE QUATERNION
      tf2::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);
      // Roll, Pitch and Yaw from rotation matrix
      double roll, pitch;
      m.getRPY(roll, pitch, r_pose1_.yaw); //Euler angles in rad
    }

    void odom_callback2(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      r_pose2_.x = msg->pose.pose.position.x;
      r_pose2_.y = msg->pose.pose.position.y;
      r_pose2_.z = msg->pose.pose.position.z;
      
      //GET THE EULER ANGLES FROM THE QUATERNION
      tf2::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);
      // Roll, Pitch and Yaw from rotation matrix
      double roll, pitch;
      m.getRPY(roll, pitch, r_pose2_.yaw); //Euler angles in rad
    }

    void movement1(float x, float y, float z, float turn)
    { //Function to assign control signals (Vx, Vy, Vz, Wz)
      vel_msg1_.linear.x = x;   vel_msg1_.linear.y = y;   vel_msg1_.linear.z = z;   vel_msg1_.angular.z = turn;
    }

    void movement2(float x, float y, float z, float turn)
    { //Function to assign control signals (Vx, Vy, Vz, Wz)
      vel_msg2_.linear.x = x;   vel_msg2_.linear.y = y;   vel_msg2_.linear.z = z;   vel_msg2_.angular.z = turn;
    }

    void leader_vel_control() //Function to generate the desired trajectory and to compute the signals control of the leader)
    { 
      double Vx, Vy, Vz, Wz; //Define control signals of the drone
      double ex, ey, ez, e_yaw; //Tracking errors
      
      //Desired positions, desired orientation and time derivative, respectively
      double Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; 

      //Desired trajectory: Helix
      double X0 = 0, Y0 = 0, Z0 = takeoff_alt, radius = 3, w = 2*M_PI/T, Vzd = 0.02;
      
      double t = now().seconds()-t0_; //Compute the time since this node started

      //Desired position in the 3D space
      Xd = X0+radius*sin(w*t);
      Yd = Y0+radius*cos(w*t);
      Zd = Z0+Vzd*t; //linear increasing
      Yawd = sin(w*t); //sinusoidal behavior

      //Corresponding time derivatives
      Xdp = radius*w*cos(w*t);
      Ydp = -radius*w*sin(w*t);
      Zdp = Vzd;
      Yawdp = w*cos(w*t);

      ex = r_pose1_.x-Xd; ey = r_pose1_.y-Yd; ez = r_pose1_.z-Zd; e_yaw = r_pose1_.yaw - Yawd; //Compute tracking errors

      //Kinematic controller. Auxiliary controls, in global coordinates
      double Ux = Xdp-k*ex;
      double Uy = Ydp-k*ey;
  
      //Translational velocities with respect to the robot frame (rotation around Z axis)
      Vx = Ux*cos(r_pose1_.yaw)+Uy*sin(r_pose1_.yaw);
      Vy = -Ux*sin(r_pose1_.yaw)+Uy*cos(r_pose1_.yaw);

      //Kinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
      Vz = Zdp-k*ez;
      Wz = Yawdp-k*e_yaw;

      //Velocities saturation
      if( abs(Vx)>Vxy_max ){ Vx = Vxy_max*abs(Vx)/Vx; }
      if( abs(Vy)>Vxy_max ){ Vy = Vxy_max*abs(Vy)/Vy; }
      if( abs(Vz)>Vz_max ){ Vz = Vz_max*abs(Vz)/Vz; }
      if( abs(Wz)>Wz_max ){ Wz = Wz_max*abs(Wz)/Wz; }

      movement1(Vx,Vy,Vz,Wz);
      //Assignation of leader velocities required by the follower robot
      Vxl_ = Vx;
      Vyl_ = Vy;
      Vzl_ = Vz;
      Wzl_ = Wz;
    }

    void follower_vel_control() //Function to compute the signals control of the follower robot
    { 
      //Note: desired follower position and yaw orientation are given by the leader's ones
      double Vxf, Vyf, Vzf, Wzf;
      double exf, eyf, ezf, e_yawf;
      double Xfd, Yfd, Xfdp, Yfdp, xlp, ylp, Ux, Uy;

      //Follower desired position with respect to the global frame
      Xfd = r_pose1_.x+s_d*cos(r_pose1_.yaw+alp_d);
      Yfd = r_pose1_.y+s_d*sin(r_pose1_.yaw+alp_d);

      //Translational velocities of the leader drone in the global frame
      xlp = Vxl_*cos(r_pose1_.yaw)-Vyl_*sin(r_pose1_.yaw);
      ylp = Vxl_*sin(r_pose1_.yaw)+Vyl_*cos(r_pose1_.yaw);

      //Corresponding desired time derivatives
      Xfdp = xlp-Wzl_*s_d*sin(r_pose1_.yaw+alp_d);
      Yfdp = ylp+Wzl_*s_d*cos(r_pose1_.yaw+alp_d);

      //Compute tracking errors (with respect to the global frame)
      exf = r_pose2_.x-Xfd;
      eyf = r_pose2_.y-Yfd; 
      ezf = r_pose2_.z-(r_pose1_.z+s_zd);  //Both the altitude and the orientation are equal to leader drone's ones
      e_yawf = r_pose2_.yaw - r_pose1_.yaw;

      //Kinematic controller. Auxiliary controls, in global coordinates
      Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf;
      
      //Translational velocities with respect to the robot frame
      Vxf = Ux*cos(r_pose2_.yaw)+Uy*sin(r_pose2_.yaw);
      Vyf = -Ux*sin(r_pose2_.yaw)+Uy*cos(r_pose2_.yaw);

      //Vertical and rotational velocities
      Vzf = Vzl_-k*ezf;
      Wzf = Wzl_-k*e_yawf;

      //Velocities saturation
      if( abs(Vxf)>Vxy_max ){Vxf = Vxy_max*abs(Vxf)/Vxf; }
      if( abs(Vyf)>Vxy_max ){Vyf = Vxy_max*abs(Vyf)/Vyf; }
      if( abs(Vzf)>Vz_max ){Vzf = Vz_max*abs(Vzf)/Vzf; }
      if( abs(Wzf)>Wz_max ){Wzf = Wz_max*abs(Wzf)/Wzf; }

      movement2(Vxf,Vyf,Vzf,Wzf);
}

    void loop()
    {
      if(mode_ == 0)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Desired takeoff altitude = %.2f", takeoff_alt);
        RCLCPP_INFO_ONCE(this->get_logger(), "Taking off ...");

        if(r_pose1_.z >= takeoff_alt || r_pose2_.z >= takeoff_alt){
          mode_ = 1;
          t0_ = now().seconds(); //Get the initial time for tracking
        }else{
          movement1(0,0,0.2,0);
          movement2(0,0,0.2,0);
        } 
      }

      if(mode_ == 1) //leader-follower scheme
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Tracking trajectory ...");
        leader_vel_control();
        follower_vel_control();
      }

      if(mode_ == 2){ //land
        RCLCPP_INFO_ONCE(this->get_logger(), "Landing ...");
        movement1(0,0,-0.2,0);
        movement2(0,0,-0.2,0);
        if(r_pose1_.z < 0.2) movement1(0,0,0,0); //Stop robot to wait for the other
        if(r_pose2_.z < 0.2) movement2(0,0,0,0);
        
        if(r_pose1_.z < 0.2 && r_pose2_.z < 0.2) rclcpp::shutdown(); //Terminate this node
      }
      
      //Display some variables of interest
      if(freq_div_ >= 25 && mode_ == 1)
      {
        // RCLCPP_INFO(this->get_logger(), "");
        freq_div_ = 0;
        
      }else freq_div_++;

      //Check for any key press
      if(kbhit())	key = getchar();
      if(key == 27) mode_ = 2; //ESC key = 27 in Ascii code

      publisher1_->publish(vel_msg1_); //Publish robot velocities
      publisher2_->publish(vel_msg2_);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_, publisher2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber1_, subscriber2_;
   
    geometry_msgs::msg::Twist vel_msg1_, vel_msg2_; //Create a msg obj to publish velocity
    double Vxl_, Vyl_, Vzl_, Wzl_; //Signals control of the leader required by the follower

    std::string ns1_, ns2_; 
    
    int freq_div_, mode_ = 0; //mode = 0 taking off, 1 trajectory tracking, 2 landing
    robot_pose r_pose1_, r_pose2_; //drones posture variables
    double t0_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  RCLCPP_INFO(logger, "argc = %d", argc);
  // Obtain parameters from command line arguments

  if (argc < 3) //node_name and ns1, ns2 = 3; if this node is run within a launcher, node_name and ns1, ns2 --ros-args = 4
  {
    RCLCPP_ERROR(logger, "Robots namespace not given\nUsage: "
      "$ ros2 run my_uavs two_quadrotors_lf r1 r2");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LFController>(argv);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


