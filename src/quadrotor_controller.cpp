/*
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

$ ros2 run my_uavs quadrotor_controller --ros-args -r __ns:=/r1
	# Runs this node. NOTE: Do not run this node within a launch file
*/

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//Includes needed by keyboard events
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

double takeoff_alt = 1.2; //Desired takeoff altitude
int key;

double Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4; //maximum translational and rotational velocities, in [m/s] and [rad/s]
double T = 100, k = 0.4; //Trajectory period, controller gains kx = ky = k
double ex, ey, ez, e_yaw; //Tracking errors

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

class TrajControl : public rclcpp::Node
{
  public:
    TrajControl() : Node("quadrotor_controller")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);

      timer_ = this->create_wall_timer(20ms, std::bind(&TrajControl::loop, this)); //20ms = 50 Hz
      
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 
            2, std::bind(&TrajControl::odom_callback, this, _1));

      RCLCPP_WARN(this->get_logger(), "Node has been initialized");
      RCLCPP_INFO(this->get_logger(), "Press 'ESC' to land the drone and to finish the node");
    }
    
    ~TrajControl() //Class destructor definition
    {
      RCLCPP_INFO(this->get_logger(), "Node has been terminated"); 
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      x_ = msg->pose.pose.position.x;
      y_ = msg->pose.pose.position.y;
      z_ = msg->pose.pose.position.z;
      
      //GET THE EULER ANGLES FROM THE QUATERNION
      tf2::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll, Pitch and Yaw from rotation matrix
      double roll, pitch;

      m.getRPY(roll, pitch, yaw_); //Euler angles in rad
    }

    void movement(float x, float y, float z, float turn)
    { //Function to assign control signals (Vx, Vy, Vz, Wz)
      vel_msg_.linear.x = x;
      vel_msg_.linear.y = y;
      vel_msg_.linear.z = z;
      vel_msg_.angular.z = turn;
    }

    void velocity_controller() //Function to generate the desired trajectory and to compute the signals control
    { 
      double Vx, Vy, Vz, Wz; //Define control signals of the drone
      
      //Desired positions, desired orientation and time derivative, respectively
      double Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; 

      //Desired trajectory: Lemniscate
      double a = 3, b = 1.5, X0 = 0, Y0 = -0.5, Z0 = 1.5, w = 2*M_PI/T, c = 0.5, d = M_PI/2;
      
      double t = now().seconds()-t0_; //Compute the time since this node started

      //Desired position in the 3D space
      Xd = X0+a*sin(w*t);
      Yd = Y0+b*sin(2*w*t);
      Zd = Z0+c*sin(w*t); //Note: 1 <= Zd <= 2
      Yawd = d*sin(w*t);

      //Corresponding time derivatives
      Xdp = a*w*cos(w*t);
      Ydp = 2*b*w*cos(2*w*t);
      Zdp = c*w*cos(w*t);
      Yawdp = d*w*cos(w*t);

      ex = x_-Xd; ey = y_-Yd; ez = z_-Zd; e_yaw = yaw_ - Yawd; //Compute tracking errors

      //Kinematic controller. Auxiliary controls, in global coordinates
      double Ux = Xdp-k*ex;
      double Uy = Ydp-k*ey;
  
      //Translational velocities with respect to the robot frame (rotation around the Z axis)
      Vx = Ux*cos(yaw_)+Uy*sin(yaw_);
      Vy = -Ux*sin(yaw_)+Uy*cos(yaw_);

      //Kinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
      Vz = Zdp-k*ez;
      Wz = Yawdp-k*e_yaw;

      //Velocities saturation
      if( abs(Vx)>Vxy_max ){ Vx = Vxy_max*abs(Vx)/Vx; }
      if( abs(Vy)>Vxy_max ){ Vy = Vxy_max*abs(Vy)/Vy; }
      if( abs(Vz)>Vz_max ){ Vz = Vz_max*abs(Vz)/Vz; }
      if( abs(Wz)>Wz_max ){ Wz = Wz_max*abs(Wz)/Wz; }

      movement(Vx,Vy,Vz,Wz);
      // publisher_->publish(vel_msg_);
    }

    void loop()
    {
      if(mode_ == 0)
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Desired takeoff altitude = %.2f\n Taking off ...", takeoff_alt);

        if(z_ >= takeoff_alt){
          mode_ = 1;
          t0_ = now().seconds(); //Get the initial time for tracking
        }else{
          movement(0,0,0.2,0);
        } 
      }

      if(mode_ == 1) //trajectory tracking
      { 
        RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory tracking ...");
        velocity_controller();
      }
      if(mode_ == 2){ //land
        RCLCPP_INFO_ONCE(this->get_logger(), "Landing ...");
        movement(0,0,-0.2,0);
        if(z_ < 0.2) rclcpp::shutdown(); //Terminate this node
      }
      
      //Display the tracking errors
      if(freq_div_ >= 25 && mode_ == 1)
      {
        RCLCPP_INFO(this->get_logger(), "ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f", ex,ey,ez,e_yaw);
        freq_div_ = 0;
      }else freq_div_++;

      //Check a key press
      if(kbhit())	key = getchar();
      if(key == 27 || key == 'q' || key == 'Q') mode_ = 2; //ESC key = 27 in Ascii code

      publisher_->publish(vel_msg_); //Publish the 4 control signals
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
   
    geometry_msgs::msg::Twist vel_msg_; //Create a msg obj to publish velocity
    
    int freq_div_, mode_ = 0; //mode = 0 taking off, 1 trajectory tracking, 2 landing
    double x_, y_, z_, yaw_; //drone posture variables
    double t0_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajControl>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


