#include "navigation/drc_controller.hpp" 

Controller::Controller : Node("controller") { 
  cmd_vel_sub_ = create_subscription<Twist>("cmd_vel", 10, std::bind(&Controller::cmd_vel_cb, this _1)); 
  speed_pub_ = create_publisher<Int32>("speed", 10);
  steer_pub_ = create_publisher<Int32>("steer", 10);  
} 

Controller::cmd_vel_cb(Twist::SharedPtr msg) { 
  Int32 speed_msg, steer_msg; 
  speed_msg.data = msg->linear.x; 
  steer_msg.data = msd->angular.z; 
  speed_pub_->publish(speed_msg); 
  steer_pub_->publish(steer_msg);
}
