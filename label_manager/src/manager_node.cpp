/*
 * manager_node.cpp
 *
 */

#include <ros/ros.h>
#include "label_manager/manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "label_manager");
    ros::NodeHandle nodeHandle("~");

    label_manager::LabelManager lm(nodeHandle);

    ros::spin();
    return 0;
}
