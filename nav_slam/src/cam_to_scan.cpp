#include "cam_to_scan.hpp"

CamToScan::CamToScan() : rclcpp::Node("cam_to_scan") {
    RCLCPP_INFO(this->get_logger(), "Initialised cam depth node");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CamToScan>());
    rclcpp::shutdown();
    return 0;
}
