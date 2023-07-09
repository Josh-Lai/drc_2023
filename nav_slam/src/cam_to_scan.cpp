#include "cam_to_scan.hpp"

using std::placeholders::_1;
CamToScan::CamToScan() : rclcpp::Node("cam_to_scan") {
    RCLCPP_INFO(this->get_logger(), "Initialised cam depth node");

    camDepthSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points",
            10, 
            std::bind(&CamToScan::point_cloud_cb, this, _1)
    );

    //Send to cloud_in, to be converted to the LaserScan topic
    cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 10);
}

void CamToScan::point_cloud_cb(sensor_msgs::msg::PointCloud2 msg) {
    cloudPub_->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CamToScan>());
    rclcpp::shutdown();
    return 0;
}
