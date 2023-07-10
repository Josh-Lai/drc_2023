#ifndef CAMERA_CORRECTOR_HPP
#define CAMERA_CORRECTOR_HPP

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"


class CameraCorrector : public rclcpp::Node {
    public:
        CameraCorrector();
    private:
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr alignedCamSub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr correctedCamPub_;

        void correct_cam_info(sensor_msgs::msg::CameraInfo msg);
};

#endif
