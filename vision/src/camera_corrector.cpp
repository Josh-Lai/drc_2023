#include "camera_corrector.hpp"

CameraCorrector::CameraCorrector() : rclcpp::Node("cam_correction_node") {
    RCLCPP_INFO(this->get_logger(), "Initialised Camera Info Correction Node");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCorrector>());
    rclcpp::shutdown();
    return 0;
}
