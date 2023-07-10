#include "laser_scan_summer.hpp"

using std::placeholders::_1;
LaserScanSummer::LaserScanSummer() : rclcpp::Node("laser_scan_summer") {
    scan1Sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/depthscan", 10, std::bind(&LaserScanSummer::scan1_cb, this, _1)
    );

    scan2Sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lanescan", rclcpp::QoS(5).best_effort(), std::bind(&LaserScanSummer::scan2_cb, this,_1)
    );
    
    scan3Pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("fullscan", 10);
    RCLCPP_INFO(this->get_logger(), "init laser scan summer");
    
}

void LaserScanSummer::scan1_cb(sensor_msgs::msg::LaserScan msg) {
    scan1_ = msg;
    update_scan3();
    return;
}

void LaserScanSummer::scan2_cb(sensor_msgs::msg::LaserScan msg) {
    scan2_ = msg;
    update_scan3();
    return;
}

void LaserScanSummer::update_scan3(void) {
    //Handle empty scans
    
    if(scan1_.ranges.size() == 0) {
        scan3Pub_->publish(scan2_);
    } else if(scan2_.ranges.size() == 0) {
        scan3Pub_->publish(scan1_);
    }
    //Loop over the lanes (Scan2) if there is no entry, check for box
    sensor_msgs::msg::LaserScan scanFull = scan2_;
    int numRanges = scan2_.ranges.size();
    if (numRanges == 0) {
        return;
    }
    for (int i = 0; i < numRanges; i++) {
        if(std::isnan(scanFull.ranges[i])) {
            scanFull.ranges[i] = scan1_.ranges[i];
        } else if(scan1_.ranges[i] < scan2_.ranges[i]) {
            scanFull.ranges[i] = scan1_.ranges[i];
        }

    }
    scanFull.header = scan1_.header;
    //publish the full scan
    scan3Pub_->publish(scanFull);
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSummer>());
    rclcpp::shutdown();
    return 0;
}
