#ifndef CAM_TO_SCAN
#define CAM_TO_SCAN

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @brief Processes Camera data to pass to SLAM toolboxk
 */
class CamToScan : public rclcpp::Node {
    public: 
        CamToScan();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camDepthSub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
        void point_cloud_cb(sensor_msgs::msg::PointCloud2 msg);
};


#endif
