#ifndef LOCALISATION_HPP
#define LOCALISATION_HPP

#include <memory>
#include <iostream>
#include <cassert>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
        
#include "pcl/common/common.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"


#include "pcl_conversions/pcl_conversions.h"


class Localisation : public rclcpp::Node {
    public:
        Localisation();
        
    private: 
        /**
         * @brief Runs the Simple SLAM implementation to obtain the local transformation
         * between the last transform and the newest
         *
         * @param newPointCloud New sensor reading 
         *
         * @return relTransform x,y,z,roll, pitch,yaw Parameters descibing the relative transform
         */
        std::vector<float> get_local_transform(sensor_msgs::msg::PointCloud2 newPointCloud);
        
        /** 
         *  @brief create a hypothesis grid based on max min and delta
         *  
         *  @note Will be a grid of 6 DOF space
         */

        std::vector<float> generate_hypothesis_space(std::vector<float> min, std::vector<float> delta, std::vector<float> max);

        /**
         *  @brief constructs a flattened vector ranging from min to max in delta increments
         */

        void construct_space(std::vector<float> &buffer, std::vector<float> &entry, std::vector<float> &min, std::vector<float> &max, std::vector<float> &delta, int depth); 
       

        void pointcloud2_to_pclpointcloud(sensor_msgs::msg::PointCloud2 &src, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);
    private:
        // Point cloud to identify localisation
        sensor_msgs::msg::PointCloud2 lastPointCloud_; 


};

#endif
