#include "Scan2PCL.h"
#include "lidar_detection/lidar_msg.h" 
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int32MultiArray.h"


class CloudCluster
{
public:
    CloudCluster();
    ~CloudCluster();
    void Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg);
    void publish_msg(int num, float position_x, float position_y);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_lidar_msg;
    ros::Publisher markerPub;
    lidar_detection::lidar_msg lidar_msg;
    bool *lidar_update;
    int n;
};