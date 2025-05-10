
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>




using namespace std;
ros::Publisher no_ground_pc_pub;
ros::Publisher geomtrical_filter_pc_pub;
ros::Publisher clustering_pc_pub;




void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  
  pcl::PointCloud<pcl::PointXYZ> cloud;

  BOOST_FOREACH (const pcl::PointXYZ &pt, temp_cloud->points)
  { 
    pcl::PointXYZ test;

    if (pt.x >= 0.5 && pt.x <= 20 && pt.y >= -7 && pt.y <= 7)
    {
      test.x = pt.x;
      test.y = pt.y;
      test.z = pt.z;

      cloud.points.push_back(test);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
  result=cloud.makeShared();



  sensor_msgs::PointCloud2 geometric_filtered_cloud;
  pcl::toROSMsg(*result,geometric_filtered_cloud);
  geometric_filtered_cloud.header.frame_id="velodyne";
  geomtrical_filter_pc_pub.publish (geometric_filtered_cloud);
  


  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);     //Set model type
  seg.setMethodType(pcl::SAC_RANSAC);       //Set random sampling consistency method type
  seg.setMaxIterations(3000);              //Indicates the maximum distance from the point to the estimated model,
  seg.setDistanceThreshold(0.045);           //Set the distance threshold. The distance threshold determines the condition that the point is considered to be an inside point.
  seg.setInputCloud(result);
  seg.segment(*inliers, *coefficients);


  if (inliers->indices.size() == 0)
  {
      cout<<"error! Could not found any inliers!"<<endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // extract ground
  // Extract the segmented point set on the plane from the point cloud
  pcl::ExtractIndices<pcl::PointXYZ> extractor;//Point extraction object
  extractor.setInputCloud(result);
  extractor.setIndices(inliers);
  extractor.setNegative(true);
  extractor.filter(*cloud_filtered);
  // vise-versa, remove the ground not just extract the ground
  // just setNegative to be true
  
  // cout << "filter done."<<endl;


  // Publish the data
  sensor_msgs::PointCloud2 no_ground;
  pcl::toROSMsg(*cloud_filtered,no_ground);
  no_ground.header.frame_id="velodyne";
  no_ground_pc_pub.publish(no_ground);




///////////////////////////////////////////////////////////////////////////////////////
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 2cm
  ec.setMinClusterSize (1);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  pcl::PointCloud<pcl::PointXYZ> centroids;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointXYZ point;
    double avgx = 0;
    double avgy = 0;
    int count = 0;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices){
      avgx += (*cloud_filtered)[idx].x;
      avgy += (*cloud_filtered)[idx].y;
      count++;
    }
    avgx /= count;
    avgy /= count;
    
    point.x = avgx;
    point.y = avgy;
    point.z = 0;

    centroids.points.push_back(point);
    printf("x = %lf, y = %lf\n", avgx, avgy);
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  centroid_cloud=centroids.makeShared();

  sensor_msgs::PointCloud2 centroid_cloud_result;
  pcl::toROSMsg(*centroid_cloud,centroid_cloud_result);
  centroid_cloud_result.header.frame_id="velodyne";
  clustering_pc_pub.publish (centroid_cloud_result);
///////////////////////////////////////////////////////////////////////////////////////  

  printf("\n\n");


}






int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cone_detection");
  ros::NodeHandle nh;

  
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, callback);
  no_ground_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ground_removal_points", 1);
  geomtrical_filter_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/geomtrical_filter_points", 1);
  clustering_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/clustering_points", 1);
  
  ros::spin();
}
