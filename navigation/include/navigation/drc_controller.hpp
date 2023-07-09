#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define INFO(...)  RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...)  RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)
#define DEBUG(...) RCLCPP_DEBUG(get_logger(), __VA_ARGS__)

using std::placeholders::_1;
using Int32 = std_msgs::msg::Int32;
using Twist = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

class Controller : public rclcpp::Node
{
public:
  Controller();
private:
  void cmd_vel_cb(Twist::SharedPtr msg);
private:
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<Int32>::SharedPtr speed_pub_;
  rclcpp::Publisher<Int32>::SharedPtr steer_pub_;
};

#endif/*__CONTROLLER_HPP__*/
