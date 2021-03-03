// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS headers
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <homework3/PointCloudExampleConfig.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <math.h>


// Message headers
#include <sensor_msgs/PointCloud2.h>

// to visualize vectors
#include <geometry_msgs/PoseArray.h>

//Fill in wireframes to output clusters
#include <avs_lecture_msgs/TrackedObjectArray.h>

// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

// Namespace matches ROS package name
namespace homework3 {

  class Homework3 {
    public:
      Homework3(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(PointCloudExampleConfig& config, uint32_t level);
      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

      // Pipeline stage functions

      void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);

      void mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);

      void generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);

      // Subscribers and publishers
      ros::Subscriber sub_cloud_;
      ros::Publisher pub_merged_cluster_cloud_;
      ros::Publisher pub_bboxes_;
      ros::Publisher pub_normals_;

      dynamic_reconfigure::Server<PointCloudExampleConfig> srv_;
      PointCloudExampleConfig cfg_;

      // Output messages
      sensor_msgs::PointCloud2 merged_cluster_cloud_;
      geometry_msgs::PoseArray normals_;
      avs_lecture_msgs::TrackedObjectArray bboxes_;

      // For looking up transforms
      tf2_ros::Buffer buffer_;
      tf2_ros::TransformListener listener_;

      // KD search tree object for use by PCL functions 
      // Algorithm that re-orders the 1D point cloud array and searches for neighboring points in the point cloud 
      pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;

      int kSearch_;

  };

}
