/*
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

$ ros2 run my_uavs simple_movement --ros-args -r __ns:=/r1
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

#define RAD2DEG 57.295779513
double takeoff_alt = 1.2; //Desired takeoff altitude
int key;

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

class SimpleMovement : public rclcpp::Node
{
  public:
    SimpleMovement() : Node("simple_movement")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);

      timer_ = this->create_wall_timer(50ms, std::bind(&SimpleMovement::loop, this)); //50ms = 20 Hz
      
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 
            2, std::bind(&SimpleMovement::odom_callback, this, _1));

      RCLCPP_WARN(this->get_logger(), "Node has been initialized");
      RCLCPP_INFO(this->get_logger(), "Press 'ESC' to land the drone and finish the node");
    }
    
    ~SimpleMovement() //Class destructor definition
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
      double roll, pitch, yaw;

      m.getRPY(roll, pitch, yaw); //Euler angles in rad
      roll_deg_ = roll * RAD2DEG;
      pitch_deg_ = pitch * RAD2DEG;
      yaw_deg_ = yaw * RAD2DEG;
      
      /*
      //GET THE QUATERNION FROM THE EULER ANGLES
      tf2::Quaternion quat;
      quat.setRPY(roll,pitch,yaw);
      std::cout << "quat: " << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << std::endl;    
      */
    }

    void movement(float x, float y, float z, float turn)
    { //Function to assign control signals (Vx, Vy, Vz, Wz)
      vel_msg_.linear.x = x;
      vel_msg_.linear.y = y;
      vel_msg_.linear.z = z;
      vel_msg_.angular.z = turn;
    }

    void loop()
    {
      if(mode_ == 0)
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Desired takeoff altitude = %.2f\n Taking off ...", takeoff_alt);

        if(z_ >= takeoff_alt){
          mode_ = 1;
        }else{
          movement(0,0,0.2,0);
        } 
      }
      if(mode_ == 1) //Circular movement on the horizontal plane
      { 
        RCLCPP_INFO_ONCE(this->get_logger(), "Circular movement ...");
        movement(0.4,0,0,0.5);
      }
      if(mode_ == 2){ //land
        RCLCPP_INFO_ONCE(this->get_logger(), "Landing ...");
        movement(0,0,-0.2,0);
        if(z_ < 0.2) rclcpp::shutdown(); //Terminate this node
      }
      
      //Display the drone pose using a frequency divisor
      if(freq_div_ == 20)
      {
        RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, z: %.2f, R: %.2f, P: %.2f, Y: %.1f ", 
            x_,y_,z_, roll_deg_,pitch_deg_,yaw_deg_);
        freq_div_ = 0;
      }else freq_div_++;

      //Check any key press
      if(kbhit())	key = getchar();
      if(key == 27 || key == 'q' || key == 'Q') mode_ = 2; //ESC key = 27 in Ascii code

      publisher_->publish(vel_msg_); //Publish velocity
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
   
    geometry_msgs::msg::Twist vel_msg_; //Create a msg obj to publish velocity
    
    int freq_div_, mode_ = 0; //mode = 0 taking off, 1 moving around, 2 landing
    double x_, y_, z_, roll_deg_, pitch_deg_, yaw_deg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMovement>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


