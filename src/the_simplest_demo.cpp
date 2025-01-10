/*
@description:
This node publishes velocities (in open-loop) to teleoperate a the simulated drone, and it subscribes to the drone 
odometry in order to obtain its position and its orientation.
Both the translational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published on the "/r1/cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "/r1/odom" topic, and they are displayed.

-- Use ctrl - C to finish this node. Important: drone will keep moving

DIRECTIONS:
In different terminals:
$ ros2 launch my_uavs ign_world_launch.py
	# Launchs one simulated quadrotor in Ignition and runs ros_ign_bridge

$ ros2 run my_uavs the_simplest_demo
	# Runs this node
*/

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define RAD2DEG 57.295779513


class SimpleMovement : public rclcpp::Node
{
  public:
    SimpleMovement() : Node("the_simplest_demo")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/r1/cmd_vel", 2);

      timer_ = this->create_wall_timer(50ms, std::bind(&SimpleMovement::loop, this)); //50ms = 20 Hz
      
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/r1/odom", 
            2, std::bind(&SimpleMovement::odom_callback, this, _1));

      RCLCPP_WARN(this->get_logger(), "Node has been initialized");
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
      
      //GET THE EULER ANGLES FROM QUATERNION
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
    }

    void loop()
    {
      vel_msg_.linear.x = 0.4;
      vel_msg_.linear.y = 0.0;
      vel_msg_.linear.z = 0.1;
      vel_msg_.angular.z = 0.6;
      
      //Display the drone pose using a frequency divisor
      if(freq_div_ == 20)
      {
        RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, z: %.2f, R: %.2f, P: %.2f, Y: %.2f", 
            x_,y_,z_, roll_deg_,pitch_deg_,yaw_deg_);
        freq_div_ = 0;
      }else freq_div_++;

      publisher_->publish(vel_msg_); //Publish velocity
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
   
    geometry_msgs::msg::Twist vel_msg_; //Create a msg obj to publish velocity
    
    int freq_div_;
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


