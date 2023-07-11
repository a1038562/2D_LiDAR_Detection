#include "Cluster.h"

CloudCluster::CloudCluster(): n(6)
{
    lidar_update = new bool[n];
    sub = nh.subscribe("/pcl2", 1, &CloudCluster::Callback, this); // PCL2 Subscribe
    pub_lidar_msg = nh.advertise<lidar_detection::lidar_msg>("/lidar_info", 1); // Lidar Info Publish
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("/lidar_detect", 1); // Marker Publish
}

CloudCluster::~CloudCluster()
{
    delete[] lidar_update;
}

void CloudCluster::publish_msg(int num, float position_x, float position_y) 
{
    lidar_msg.position_x[num] = position_x;
    lidar_msg.position_y[num] = position_y;
    lidar_update[num] = true;
}
    
void CloudCluster::Callback(const sensor_msgs::PointCloud2ConstPtr& pcl2_msg)
{
    for (int i = 0; i < n; i++)
    {
        lidar_update[i] = false;
    }
    
    std::vector<geometry_msgs::Point> center;
    visualization_msgs::MarkerArray clusterMarkers;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::fromROSMsg(*pcl2_msg, *input_cloud); // ROS Message cloud2를 cloud로 변환  
    tree->setInputCloud(input_cloud); 

    // Euclidean Cluster Extraction
    ec.setClusterTolerance(1);    // Point 사이 최소 허용 거리
    ec.setMinClusterSize(3);    // Min Cluster Size
    ec.setMaxClusterSize(100);   // Max Cluster Size
    ec.setSearchMethod(tree);   // Search Method - tree 
    ec.setInputCloud(input_cloud);    // Clustering result
    ec.extract(cluster_indices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) 
        {
            cloud_cluster->points.push_back(input_cloud->points[*pit]);
            x += input_cloud->points[*pit].x;
            y += input_cloud->points[*pit].y;
            numPts++;
        }

        geometry_msgs::Point pt;
        pt.x = x / numPts;
        pt.y = y / numPts;
        pt.z = 0.0;

        if (pt.x != 0 && pt.y != 0)
        {
            center.push_back(pt);
        }
        
    }

    if (center.size()!=0)
    {
        visualization_msgs::Marker m;
        for (int i = 0; i < n; i++) 
        {
            m.id = i;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.header.frame_id = "map";
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color.a = 1.0;
            m.color.r = 1;
            m.color.g = 0;
            m.color.b = 0;

            geometry_msgs::Point clusterC(center[i]);

            if (abs(clusterC.x) < 5.0 && abs(clusterC.y) < 5.0)
            {
                m.pose.position.x = clusterC.x;
                m.pose.position.y = clusterC.y;
                m.pose.position.z = clusterC.z;
            }
        
            if (abs(m.pose.position.x) > 0.01 && abs(m.pose.position.y) > 0.01 && abs(m.pose.position.x) < 5.0 && abs(m.pose.position.y) < 5.0)
            {
                m.action = visualization_msgs::Marker::ADD;
                clusterMarkers.markers.push_back(m);     
                publish_msg(i, m.pose.position.x, m.pose.position.y);
            }

            else
            {
                m.action = visualization_msgs::Marker::DELETE;
                clusterMarkers.markers.push_back(m);  
            }
        }
    }

    for (int i = 0; i < n; i++)
    {
        lidar_msg.lidar_update[i] = lidar_update[i];
    }

    pub_lidar_msg.publish(lidar_msg);
    markerPub.publish(clusterMarkers);

    ros::Rate loop_rate(10);
    loop_rate.sleep(); 
}
