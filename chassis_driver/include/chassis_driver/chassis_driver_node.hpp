#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

#include "chassis_driver/visibility_control.h"

namespace chassis_driver{

class ChassisDriver : public rclcpp::Node {
public:
    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscription_selfpose;

  void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
  void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);

  void rotate_vector(const double vx, const double vy, const double theta);
  void send_motorvel(const int motor_num, const double vel);
  void _subscriber_callback_selfpose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;

  rclcpp::QoS _qos = rclcpp::QoS(10);

  // 定数
  const double wheel_radius;
  const double attached_direction;
  const uint32_t stop_canid;
  const uint32_t restart_canid;
  const uint32_t velocity_canid;
  const double linear_max_vel;
  const double angular_max_vel;
  const double sqrt2over2;

  // 変数
  geometry_msgs::msg::Twist prime_vel;
  double current_yaw = 0.0;

  // 動作モード
  enum class Mode{
    cmd,
    stay,
    stop
  } mode = Mode::stay;

};

}  // namespace chassis_driver
