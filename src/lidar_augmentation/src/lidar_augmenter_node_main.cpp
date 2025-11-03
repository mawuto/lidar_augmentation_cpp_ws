// src/cpp/lidar_augmenter_node_main.cpp
#include "lidar_augmentation/lidar_augmenter_node.h"
#include <ros/ros.h>
#include <pcl/console/print.h>

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lidar_augmenter_cpp");

    // Suppress PCL "Failed to find match for field 'timestamp'" warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    try
    {
        // Create and run the augmenter node
        lidar_augmentation::LidarAugmenterNode node;
        node.run();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("LiDAR Augmenter Node failed: " << e.what());
        return 1;
    }

    return 0;
}
