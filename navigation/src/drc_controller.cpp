#include "navigation/drc_controller.hpp" 

Controller::Controller() : Node("controller") { 
  cmd_vel_sub_ = create_subscription<Twist>("cmd_vel", 10, std::bind(&Controller::cmd_vel_cb, this, _1)); 
  speed_pub_ = create_publisher<Int32>("speed_command", 10);
  steer_pub_ = create_publisher<Int32>("steer_command", 10);  
} 

void Controller::cmd_vel_cb(Twist::SharedPtr msg) { 
  Int32 speed_msg, steer_msg; 
  speed_msg.data = (int) (msg->linear.x * 100); 
  INFO("%d\n", speed_msg.data);
  steer_msg.data = (int) (msg->angular.z * 100); 
  speed_pub_->publish(speed_msg); 
  steer_pub_->publish(steer_msg);
}

int main(int argc, char** argv) { 
  rclcpp::init(argc, argv); 
  rclcpp::spin(std::make_shared<Controller>()); 
  rclcpp::shutdown();
  return 0; 
}
