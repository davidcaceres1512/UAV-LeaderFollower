#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define RAD2DEG 57.295779513
double takeoff_alt = 1.2; // Desired takeoff altitude
int key;

class CommandControlledMovement : public rclcpp::Node
{
public:
    CommandControlledMovement() : Node("command_controlled_movement")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/drone/commands", 10, std::bind(&CommandControlledMovement::command_callback, this, _1));

        timer_ = this->create_wall_timer(50ms, std::bind(&CommandControlledMovement::loop, this)); // 50ms = 20 Hz
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, std::bind(&CommandControlledMovement::odom_callback, this, _1));

        RCLCPP_WARN(this->get_logger(), "Node has been initialized");
        RCLCPP_INFO(this->get_logger(), "Ready to receive commands");
    }

    ~CommandControlledMovement()
    {
        RCLCPP_INFO(this->get_logger(), "Node has been terminated");
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto command = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

        if (command == "TAKEOFF_START")
        {
            mode_ = 0; // Set mode to takeoff
        }
        else if (command == "LAND")
        {
            mode_ = 2; // Set mode to land
        }
        else if (command == "ROTATE")
        {
            rotating_ = true; // Activate rotation
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        // Get Euler angles from quaternion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // Euler angles in rad
        roll_deg_ = roll * RAD2DEG;
        pitch_deg_ = pitch * RAD2DEG;
        yaw_deg_ = yaw * RAD2DEG;
    }

    void movement(float x, float y, float z, float turn)
    {
        vel_msg_.linear.x = x;
        vel_msg_.linear.y = y;
        vel_msg_.linear.z = z;
        vel_msg_.angular.z = turn;
    }

    void loop()
    {
        if (mode_ == 0) // Takeoff
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Taking off...");
            if (z_ >= takeoff_alt)
            {
                mode_ = 1; // Transition to hover mode
            }
            else
            {
                movement(0, 0, 0.2, 0);
            }
        }
        else if (mode_ == 1) // Hover or rotate
        {
            if (rotating_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Rotating...");
                movement(0, 0, 0, 0.5); // Rotate in place
            }
            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Hovering...");
                movement(0, 0, 0, 0); // Hover
            }
        }
        else if (mode_ == 2) // Land
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Landing...");
            if (z_ < 0.2)
            {
                movement(0, 0, 0, 0); // Stop all movement
                RCLCPP_INFO(this->get_logger(), "Landed");
            }
            else
            {
                movement(0, 0, -0.2, 0); // Descend
            }
        }

        publisher_->publish(vel_msg_); // Publish velocity
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

    geometry_msgs::msg::Twist vel_msg_;
    int mode_ = 1; // Default to hover mode
    bool rotating_ = false;

    double x_, y_, z_, roll_deg_, pitch_deg_, yaw_deg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandControlledMovement>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

