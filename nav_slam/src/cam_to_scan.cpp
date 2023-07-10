#include "cam_to_scan.hpp"

using std::placeholders::_1;
CamToScan::CamToScan() : rclcpp::Node("cam_to_scan") {
    RCLCPP_INFO(this->get_logger(), "Initialised cam depth node");

    camDepthSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lane_points",
            rclcpp::QoS(5).best_effort(),
            std::bind(&CamToScan::point_cloud_cb, this, _1)
    );

    //Send to cloud_in, to be converted to the LaserScan topic
    cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lane_points_flat", rclcpp::QoS(12).best_effort());
}

void CamToScan::point_cloud_cb(sensor_msgs::msg::PointCloud2 msg) {
    //Convert the point cloud into a 2D point cloud
    //
    //Iterate over all the points, set them at y = 0;
    pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    pcl::PointXYZ pclPoint;
    for(; iter_x != iter_x.end(); ++iter_x) {
        pclPoint.x = iter_x[0];
        pclPoint.y = pclHeight;
        pclPoint.z = iter_x[2];
        pclPointCloud.push_back(pclPoint);
    }
    //Convert this into PointCloud2
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(pclPointCloud, out);
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = msg.header.frame_id;

    
    cloudPub_->publish(out);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CamToScan>());
    rclcpp::shutdown();
    return 0;
}
