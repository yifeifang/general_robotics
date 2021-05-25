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
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/pcl_config.h>

#include <math.h>
#include "sensor_msgs/JointState.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// not good practice
ros::Publisher pub_object;
ros::Publisher pub_plane;
double head_tilt = 0;

void update_head_angle(const sensor_msgs::JointStateConstPtr & msg)
{
  // get the head tilt angle not sure if the index is fixed?
  head_tilt = msg->position[5];
}

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
  double y = 1.0 * cos(head_tilt);
  double z = 1.0 * sin(head_tilt);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,y,z);
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

  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
  double z_min = 0.05, z_max = 0.5; // we want the points above the plane, no farther than 5 cm from the surface
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ConvexHull<pcl::PointXYZ> hull;
  // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
  hull.setInputCloud(cloud_cluster);
  hull.reconstruct(*hull_points);
  if (hull.getDimension () == 2)
  {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (cloud_filtered);
    prism.setInputPlanarHull (hull_points);
    prism.setHeightLimits (z_min, z_max);
    prism.segment(*object_indices);
  }
  else
  {
    PCL_ERROR ("The input cloud does not represent a planar surface.\n");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = object_indices->indices.begin (); pit != object_indices->indices.end (); ++pit)
    object_cloud->push_back ((*cloud_filtered)[*pit]); //*
  object_cloud->width = object_cloud->size ();
  object_cloud->height = 1;
  object_cloud->is_dense = true;

  std::cout << "Object cloud: " << object_cloud->size () << " data points." << std::endl;

  sensor_msgs::PointCloud2 object_output;
  pcl::toROSMsg(*object_cloud, object_output);
  object_output.header.frame_id = "head_camera_rgb_optical_frame"; 

  sensor_msgs::PointCloud2 plane_output;
  pcl::toROSMsg(*cloud_cluster, plane_output);
  plane_output.header.frame_id = "head_camera_rgb_optical_frame"; 

  pub_object.publish(object_output);
  pub_plane.publish(plane_output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  pub_object = nh.advertise<PointCloud> ("points2_object", 1);
  pub_plane = nh.advertise<PointCloud> ("points2_plane", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_registered/points", 1, callback);
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, update_head_angle);
  ros::spin();
}