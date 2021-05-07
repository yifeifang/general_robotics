#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// not good practice
ros::Publisher pub;

void callback(const PointCloud::ConstPtr& cloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
    cloud_cluster->push_back ((*cloud)[*pit]); //*
  cloud_cluster->width = cloud_cluster->size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
  // std::stringstream ss;
  // ss << "cloud_cluster_0.pcd";
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_cluster, output);
  output.header.frame_id = "head_camera_link"; 

  // cloud_cluster->header.frame_id = "head_camera_link"; // probably head camera
  // cloud_cluster->height = cloud_cluster->width = 1;
  // pcl_conversions::toPCL(ros::Time::now(), cloud_cluster->header.stamp);

  // PointCloud::Ptr msg (new PointCloud);
  // msg->header.frame_id = "some_tf_frame";
  // msg->height = msg->width = 1;
  // msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  pub.publish (output);
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  pub = nh.advertise<PointCloud> ("points2", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/head_camera/depth_registered/points", 1, callback);
  ros::spin();
}