/**
 * @file main.cpp
 * @brief Main entry point for VEX Serial Node
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tmr_vex_serial/vex_serial_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<tmr_vex_serial:: VexSerialNode>();
    
    RCLCPP_INFO(node->get_logger(), "VEX Serial Node starting...");
    
    rclcpp::spin(node);
    
    rclcpp:: shutdown();
    return 0;
}