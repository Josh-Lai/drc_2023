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
   // minBound << -0.5, -0.5, -0.5, -5, -5, -5 << std::endl;
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
