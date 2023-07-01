#include "localisation.hpp"

Localisation::Localisation() : rclcpp::Node("localisation"){
    RCLCPP_INFO(this->get_logger(), "Initialised Localisation Node");
    return;
}

std::vector<float> get_local_transform(sensor_msgs::msg::PointCloud2 newPointCloud) {
    // Compare the last point cloud with the previous, maximise the evidence between the two

    //store the previous PointCloud in a K-D Tree
    //Get the hypothesis grid based on the 
    
    //For each Hypothesis:
    //For each point in newPointCloud * 
    //  Ej = evidence_to_closest_point
    //  E += 
    //
    return std::vector<float>(6);

}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localisation>());
    
    rclcpp::shutdown();
    return 0;
}
