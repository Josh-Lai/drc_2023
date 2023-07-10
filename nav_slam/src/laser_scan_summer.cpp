#include "laser_scan_summer.hpp"

using std::placeholders::_1;
LaserScanSummer::LaserScanSummer() : rclcpp::Node("laser_scan_summer") {
    RCLCPP_INFO(this->get_logger(), "init laser scan summer");
    scan1Sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/depthscan", 10, std::bind(&LaserScanSummer::scan1_cb, this, _1)
    );

    scan2Sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lanescan", 10, std::bind(&LaserScanSummer::scan2_cb, this,_1)
    );
    
}

void LaserScanSummer::scan1_cb(sensor_msgs::msg::LaserScan msg) {
    scan1_ = msg;
    return;
}

void LaserScanSummer::scan2_cb(sensor_msgs::msg::LaserScan msg) {
    scan2_ = msg;
    return;
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSummer>());
    rclcpp::shutdown();
    return 0;
}
