#include "camera_corrector.hpp"

using std::placeholders::_1;

CameraCorrector::CameraCorrector() : rclcpp::Node("cam_correction_node") {
    RCLCPP_INFO(this->get_logger(), "Initialised Camera Info Correction Node");
    alignedCamSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/aligned_depth_to_color/camera_info", 10, std::bind(&CameraCorrector::correct_cam_info, this, _1));
    correctedCamPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/corrected_align/camera_info", 10);
}

void CameraCorrector::correct_cam_info(sensor_msgs::msg::CameraInfo msg) {
    sensor_msgs::msg::CameraInfo send = msg;
    send.height /= 2;
    send.width /= 2;
    send.k[5] = send.k[5]/2;
    send.p[6] = send.p[6]/2;
    correctedCamPub_->publish(send);
    return;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCorrector>());
    rclcpp::shutdown();
    return 0;
}
