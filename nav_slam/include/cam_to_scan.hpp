#ifndef CAM_TO_SCAN
#define CAM_TO_SCAN

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief Converts the point cloud to a flattened point cloud for a scan
 */
class CamToScan : public rclcpp::Node {
    public: 
        CamToScan();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camDepthSub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
        void point_cloud_cb(sensor_msgs::msg::PointCloud2 msg);
        

        const float pclHeight = 0;
};


#endif
