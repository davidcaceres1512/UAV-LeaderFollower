/*
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

$ ros2 run my_uavs quadrotor_controller_paths_urdf --ros-args -r __ns:=/r1
	# Runs this node. NOTE: Do not run this node within a launch file

$ rviz2 -d ~/colcon_ws/src/my_uavs/rviz/one_drone.rviz
  # Open Rviz with my configuration file to visualize paths (optional, included in the launch file)

To show the node ros-params$ ros2 param dump /r1/quadrotor_controller_paths_urdf
To get k_gain param value$ ros2 param get /r1/quadrotor_controller_paths_urdf k_gain
To set k_gain param value$ ros2 param set /r1/quadrotor_controller_paths_urdf k_gain 0.2

To see the transformation between two frames
$ ros2 run tf2_ros tf2_echo r1/odom r1/base_footprint
*/

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

//Includes needed by keyboard events
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

double takeoff_alt = 1.2; //Desired takeoff altitude
int key;

//Trajectory variables
double t, t0; ///timer, initial time
double Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4; //maximum translational and rotational velocities, in [m/s] and [rad/s]

double T = 100, k; //Trajectory period, controller gains kx = ky = k
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
    TrajControl() : Node("quadrotor_controller_paths_urdf")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);

      timer_ = this->create_wall_timer(20ms, std::bind(&TrajControl::loop, this)); //20ms = 50 Hz
      
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 
            2, std::bind(&TrajControl::odom_callback, this, _1));

      this->declare_parameter<double>("k_gain", 0.4);
      get_parameter("k_gain", k);
      RCLCPP_INFO(this->get_logger(), "k_gain = k: %.2f", k);


      std::string ns = this->get_namespace();
      if (ns.size()>1){ //By default ns='/'
        RCLCPP_INFO(this->get_logger(), "The provided ns: %s", ns.c_str());
        frame_id_ = ns+frame_id_;
        child_frame_id_ = ns+child_frame_id_;
      }

      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      //Publishers to publish paths
      pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 1); //robot path
      pub_des_path_ = this->create_publisher<nav_msgs::msg::Path>("des_path", 1); //desired path

      RCLCPP_INFO(this->get_logger(), "Press 'ESC' key to land the drone and finish the node");
      RCLCPP_INFO(this->get_logger(), "Press 'c' key to clean the desired trajectory");
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


      //Create and fill out the transform broadcaster
      tranStamp_.header.stamp = this->get_clock()->now();
      tranStamp_.header.frame_id = frame_id_;
      tranStamp_.child_frame_id = child_frame_id_;

      tranStamp_.transform.translation.x = x_;
      tranStamp_.transform.translation.y = y_;
      tranStamp_.transform.translation.z = z_;

      tranStamp_.transform.rotation.x = q.x();
      tranStamp_.transform.rotation.y = q.y();
      tranStamp_.transform.rotation.z = q.z();
      tranStamp_.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tranStamp_); // Send the transformation


      //Paths
      path_msg_.header.frame_id = msg->header.frame_id;
      des_path_msg_.header.frame_id = msg->header.frame_id;

      RCLCPP_INFO_ONCE(this->get_logger(), "Robot path frame_id: %s", path_msg_.header.frame_id.c_str());
      
      robot_points_.pose.position.x = x_;
      robot_points_.pose.position.y = y_;
      robot_points_.pose.position.z = z_;
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
  
      //Translational velocities with respect to the robot frame (rotaion around Z axis)
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

      //Path
      des_points_.pose.position.x = Xd;
      des_points_.pose.position.y = Yd;
      des_points_.pose.position.z = Zd;

      path_msg_.poses.push_back(robot_points_); //Add the new point at the end of the list
      des_path_msg_.poses.push_back(des_points_);

      if(path_msg_.poses.size() > 800)
      {
        path_msg_.poses.erase(path_msg_.poses.begin()); //delete the oldest point
      }

      if(key == 'c') //des_path_msg_.poses.size() > 4000
      {
        key = -1;
        des_path_msg_.poses.clear();
        RCLCPP_INFO(this->get_logger(), "Desired path reset");
      }

      //Publish points on topics
      pub_path_->publish(path_msg_);
      pub_des_path_->publish(des_path_msg_); 
    }

    void loop()
    {
      if(mode_ == 0)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Desired takeoff altitude = %.2f", takeoff_alt);
        RCLCPP_INFO_ONCE(this->get_logger(), "Taking off ...");

        if(z_ >= takeoff_alt){
          mode_ = 1;
          t0_ = now().seconds(); //Get the initial time for tracking
        }else{
          movement(0,0,0.2,0);
        } 
      }

      if(mode_ == 1) //trajectory tracking
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Tracking trajectory ...");
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
        // RCLCPP_INFO(this->get_logger(), "ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw);
        freq_div_ = 0;
        
        get_parameter("k_gain", k); //Update k_gain param
        if(k <= 0){
          RCLCPP_WARN(this->get_logger(), "k_gain cannot be 0 or lower. k = 0.1");
          k = 0.1;
        }
        if(k > 2){
          RCLCPP_WARN(this->get_logger(), "k_gain cannot be very big. k = 2.0");
          k = 0.1;
        }
      }else freq_div_++;

      //Check a key press
      if(kbhit())	key = getchar();
      if(key == 27) mode_ = 2; //ESC key = 27 in Ascii code

      publisher_->publish(vel_msg_); //Publish the 4 control signals
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_, pub_des_path_;
   
    geometry_msgs::msg::Twist vel_msg_; //Create a msg obj to publish velocity

    std::string frame_id_ = "/odom", child_frame_id_ = "/base_footprint";
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped tranStamp_; 

    //std::string path_frame = "odom";
    nav_msgs::msg::Path path_msg_, des_path_msg_;
    geometry_msgs::msg::PoseStamped robot_points_, des_points_; //3D points
    
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


