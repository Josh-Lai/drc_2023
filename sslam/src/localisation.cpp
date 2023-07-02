#include "localisation.hpp"

Localisation::Localisation() : rclcpp::Node("localisation"){
    RCLCPP_INFO(this->get_logger(), "Initialised Localisation Node");

    std::vector<float> minBound, maxBound, deltaBound;
    minBound = {-0.5, -0.5, -0.5, -5, -5, -5};
    maxBound = {0.5, 0.5, 0.5, 5, 5, 5};
    deltaBound = {0.1,0.1, 0.1, 1, 1, 1};

    std::vector<float> poseSpace = generate_hypothesis_space(minBound, deltaBound, maxBound);
    std::string hypMsg;
    hypMsg = "Created Hypothesis Space of size ";
    hypMsg += std::to_string(poseSpace.size());

    RCLCPP_INFO(this->get_logger(), hypMsg.c_str());
    return;
}

std::vector<float> Localisation::get_local_transform(sensor_msgs::msg::PointCloud2 newPointCloud) {
    // Compare the last point cloud with the previous, maximise the evidence between the two
    
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> newPc, oldPc;
    
    //Convert the point cloud for PCL

    pointcloud2_to_pclpointcloud(newPointCloud, newPc);
    pointcloud2_to_pclpointcloud(lastPointCloud_, oldPc);
    //Check if the oldPointCloud is empty, if so then cant do anything here
    
    if( oldPc->size() == 0) {
        lastPointCloud_ = newPointCloud;
        return std::vector<float>(6,0);
    }
    
    //Otherwise, store in a KD-tree for evidence computation
    
    pcl::KdTree<pcl::PointXYZ>::Ptr kdOldPc(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdOldPc->setInputCloud(oldPc);

    //Run the Evidence computation on the old and new point cloud
    
    
    //for each Hypothesis
    //transform the point cloud based on Hypothesis
    //for each point
    //Evaluate closest evidence
    //select maximum pose
    return std::vector<float>(6);

}

void Localisation::pointcloud2_to_pclpointcloud(sensor_msgs::msg::PointCloud2 &src, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &out) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(src, pcl_pc2);
    out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(pcl_pc2, *out);
    return;
}

std::vector<float> Localisation::generate_hypothesis_space(std::vector<float> min, std::vector<float> delta, std::vector<float> max){ 
    //Output vector will be flattened for effeciency
    assert(min.size() == max.size());
    assert(delta.size() == max.size());
    std::vector<float> poseBuffer;
    
    std::vector<float> entry(6);
    construct_space(poseBuffer,entry, min, max, delta, 0);

    return poseBuffer;
}


void Localisation::construct_space(std::vector<float> &buffer, std::vector<float> &entry, std::vector<float> &min, std::vector<float> &max, std::vector<float> &delta, int depth) {
    assert(depth < (int)max.size());
    assert(depth >= 0);
    for (float i = min[depth]; i <= max[depth]; i += delta[depth]) {
        entry[depth] = i;
        if (depth == 5) {
            for (int j = 0; j < 6; j++) {
                buffer.push_back(entry[j]);
            }
        } else {
            construct_space(buffer, entry, min, max, delta, depth+1);
        }
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localisation>());
    
    rclcpp::shutdown();
    return 0;
}
