#include "Cluster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_detection");
    Scan2PCL pcl;
    CloudCluster cluster;
    ROS_INFO("Lidar Object Created");
    ros::spin();
}
