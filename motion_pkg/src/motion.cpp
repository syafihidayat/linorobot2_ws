#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
// #include "geometry_msgs/msg/twist.hpp"
#include "motion_pkg/pid.hpp"
#include "motion_pkg/convertion.hpp"

using namespace std;
using std::max;
using std::min;
using std::placeholders::_1;
PID mecha_linear;
PID mecha_angular;
Convertion convert;

class Motion_Cam : public rclcpp::Node
{
public:
  Motion_Cam() : Node("motion_cam")
  {
    odom_subs = this->create_subscription<nav_msgs::msg::Odometry>("odom", 50, std::bind(&Motion_Cam::odom_robot_callback, this, _1));
    yolo_subs = this->create_subscription<geometry_msgs::msg::Point>("coordinate_object", 10, std::bind(&Motion_Cam::yolo_callback, this, _1));
    boundingBox_subs = this->create_subscription<geometry_msgs::msg::Point>("coordinate_boundingBox", 10, std::bind(&Motion_Cam::boundingBox_callback, this, _1));
    button_subs = this->create_subscription<std_msgs::msg::Int32>("button_triangle", 10, std::bind(&Motion_Cam::triangle_callback, this, _1));

    pub_auto = this->create_publisher<geometry_msgs::msg::Pose2D>("checking_auto", 1);
    defense_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_defense", 1);
    triangle_pub = this->create_publisher<std_msgs::msg::Int32>("triangle", 1);

    // coordinate_boundingBox_received = false;
    // coordinate_object_received = false;
  }

private:
  struct Pidparams
  {
    float kp;
    float ki;
    float kd;
  };

  struct PidparamsZ
  {
    float kp;
    float ki;
    float kd;
  };

  struct velocity
  {
    float desired_linear_vel;
    float max_angular_vel;
  };

  struct Error
  {
    float x;
    float y;
    float theta;
    float distance;
    float angle;
  } error;

  struct c
  {
    float distance;
    float angle;
  } controlled;

  geometry_msgs::msg::Point coordinate_boundingBox;
  geometry_msgs::msg::Point coordinate_object;
  std_msgs::msg::Int32 button_triangle;
  bool coordinate_boundingBox_received = false;
  // bool coordinate_object_received = false;

  int last_buttonState = 0;

  void triangle_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {

    button_triangle = *msg;


    if (msg->data == 1 && last_buttonState == 0)
    {
      RCLCPP_INFO(this->get_logger(), "button has been pressed");
    }
    else
    {

      RCLCPP_INFO(this->get_logger(), "button not yet pressed ");
    }

    last_buttonState = (msg -> data == 1) ? 1: 0;
    triangle_pub->publish(*msg);
  }

  void yolo_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {

    coordinate_object = *msg;

    RCLCPP_INFO(this->get_logger(), "Topic received: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
  }

  void boundingBox_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {

    double dx = std::abs(msg->x - coordinate_boundingBox.x);
    double dy = std::abs(msg->y - coordinate_boundingBox.y);
    double threshold = 15.0;

    if (dx > threshold || dy > threshold)
    {
      coordinate_boundingBox = *msg;
      coordinate_boundingBox_received = true;
      RCLCPP_INFO(this->get_logger(), "Updated bounding box: x=%.2f, y=%.2f, z=%.0f", msg->x, msg->y, msg->z);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Ignored small change: dx=%.2f, dy=%.2f", dx, dy);
    }

    // RCLCPP_INFO(this->get_logger(), "box: x=%.2f, y=%.2f, z%0.0f", msg->x, msg->y, msg->z);
  }

  void odom_robot_callback(const nav_msgs::msg::Odometry &msg)
  {

    if (!coordinate_boundingBox_received)
    {
      RCLCPP_WARN(this->get_logger(), "no bounding box receive yet");
      return;
    }

    // bool object_valid = (this->now() - last_object_time) < object_timeout;

    // if (!object_valid){
    //   RCLCPP_WARN(this -> get_logger(), "object data expired, using last known bounding box only");

    // }

    Pidparams parameters;
    parameters.kp = 0.005;
    parameters.ki = 0.0;
    parameters.kd = 0.0;

    PidparamsZ params;
    params.kp = 0.1;
    params.ki = 0.0;
    params.kd = 0.0;

    velocity value;
    value.desired_linear_vel = 3.0;
    value.max_angular_vel = 3.0;

    double robot_x = coordinate_boundingBox.x;
    double robot_y = coordinate_boundingBox.y;

    Convertion::Quaternion odom_robot_q = {
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z};
    double odom_robot_yaw, odom_robot_pitch, odom_robot_roll;
    convert.quat_to_eular(odom_robot_q, odom_robot_yaw, odom_robot_pitch, odom_robot_roll);

    double X_pose_frame = robot_y - 360;
    double Y_pose_frame = (robot_x - 640) * -1;

    error.x = 0 - X_pose_frame;
    error.y = 0 - Y_pose_frame;
    error.theta = 0 - odom_robot_yaw;

    if (error.theta > 180)
    {
      error.theta -= 360;
    }
    else if (error.theta < -180)
    {
      error.theta += 360;
    }
    
    error.distance = sqrt(pow(error.x, 2) + pow(error.y, 2));
    error.angle = atan2(error.y, error.x);

    mecha_linear.setBaseParam(parameters.kp, parameters.ki, parameters.kd);
    mecha_angular.setHeadingParam(params.kp, params.ki, params.kd);

    controlled.distance = mecha_linear.control_base(error.distance, value.desired_linear_vel);
    controlled.angle = mecha_angular.control_base(error.theta, value.max_angular_vel);

    const float distance_deadzone = 15.0;
    if (error.distance < distance_deadzone)
    {
      error.distance = 0.0;
      controlled.distance = 0.0;
      controlled.angle = 0.0;
    }

    auto message = geometry_msgs::msg::Twist();

    if (error.distance == 0.0)
    {
      message.linear.x = 0.0;
      message.linear.y = 0.0;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Target reached. Robot stopped.");
    }
    else
    {

      message.linear.x = controlled.distance * cos(error.angle);
      message.linear.y = controlled.distance * sin(error.angle);
      message.angular.z = controlled.angle;

      RCLCPP_INFO(this->get_logger(), "Publishing Twist: x=%.2f, y=%.2f, z=%.2f", message.linear.x, message.linear.y, message.angular.z);
    }
    defense_pub->publish(message);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr yolo_subs;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr boundingBox_subs;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_subs;

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_auto;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr defense_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr triangle_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motion_Cam>());
  rclcpp::shutdown();
  return 0;
}
