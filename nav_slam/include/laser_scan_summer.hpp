#ifndef LASER_SCAN_SUM_HPP
#define LASER_SCAN_SUM_HPP

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanSummer : public rclcpp::Node {
    public:
        LaserScanSummer();
    private: 
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1Sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2Sub_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan3Pub_;

        void scan1_cb(sensor_msgs::msg::LaserScan msg);

        // When I recieve  a point cloud sum up the points
        void scan2_cb(sensor_msgs::msg::LaserScan msg);

        sensor_msgs::msg::LaserScan scan1_; //Fast
        sensor_msgs::msg::LaserScan scan2_; //Slow

};

#endif
