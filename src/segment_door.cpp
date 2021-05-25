#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/pcl_config.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// not good practice
ros::Publisher pub;

void callback(const PointCloud::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
 
  // filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  
  // Optional
  // seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
  seg.setAxis(axis);
  seg.setEpsAngle(15.0 * (M_PI / 180.0));
  
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
    cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
  cloud_cluster->width = cloud_cluster->size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
  // std::stringstream ss;
  // ss << "cloud_cluster_0.pcd";
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_cluster, output); // original cloud_cluster
  output.header.frame_id = "head_camera_rgb_optical_frame"; 

  pub.publish (output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  pub = nh.advertise<PointCloud> ("points2", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_registered/points", 1, callback);
  ros::spin();
}