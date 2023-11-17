/*
 * manager_node.cpp
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "label_manager/manager.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<label_manager::LabelManager>(
        "label_manager"));
    rclcpp::shutdown();
    return 0;
}
