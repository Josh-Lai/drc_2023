#ifndef CAM_TO_SCAN
#define CAM_TO_SCAN

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

class CamToScan : public rclcpp::Node {
    public: 
        CamToScan();
};


#endif
