#include "localisation.hpp"

Localisation::Localisation() : rclcpp::Node("localisation"){
    RCLCPP_INFO(this->get_logger(), "Initialised Localisation Node");
    return;
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localisation>());
    
    rclcpp::shutdown();
    return 0;
}
