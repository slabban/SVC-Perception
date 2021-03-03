// Header file for the class
#include "Homework3.hpp"

// Namespace matches ROS package name
namespace homework3 
{  
  // Constructor with global and private node handle arguments
  Homework3::Homework3(ros::NodeHandle& n, ros::NodeHandle& pn) :

  //listener & kd_tree initialzed in the constructor
    listener_(buffer_),
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    srv_.setCallback(boost::bind(&Homework3::reconfig, this, _1, _2));

    sub_cloud_ = n.subscribe<sensor_msgs::PointCloud2>("/cepton/points_raw", 10, &Homework3::recvCloud, this);
    pub_merged_cluster_cloud_ = n.advertise<sensor_msgs::PointCloud2>("merged_cluster_cloud", 1);
    pub_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);


    pn.param("kSearch", kSearch_, 50);
  }

    void Homework3::reconfig(PointCloudExampleConfig& config, uint32_t level)
  {
    cfg_ = config;
  }

  void Homework3::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    geometry_msgs::TransformStamped transform;

      try {
      // Attempt to look up the transform from base_footprint to ouster (Unsure if this actually works)
      transform = buffer_.lookupTransform("base_footprint", "cepton", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      // If exception is thrown, display the error message and return
      ROS_WARN_STREAM(ex.what());
      return;
    }
    // Apply coordinate frame transformation
    sensor_msgs::PointCloud2 transformed_msg;

    pcl_ros::transformPointCloud("base_footprint", *msg, transformed_msg, buffer_);

    // Copy into PCL cloud for processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //Copy transformed_msg into input_cloud
    pcl::fromROSMsg(transformed_msg, *input_cloud);

    // Run the processing pipeline
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(input_cloud, filtered_cloud);
    voxelFilter(filtered_cloud, filtered_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    normalsFilter(filtered_cloud, no_ground_cloud);

   //Return from function here if there are no points left after filtering out the ground points
    if (no_ground_cloud->points.empty()){

        //ROS_INFO("No Valid Points Detected.. Returning.. \n");
      return;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    euclideanClustering(no_ground_cloud, cluster_clouds);
    generateBoundingBoxes(cluster_clouds);
    bboxes_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

    mergeClusters(cluster_clouds);
    merged_cluster_cloud_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

    pub_merged_cluster_cloud_.publish(merged_cluster_cloud_);
    pub_bboxes_.publish(bboxes_);
    pub_normals_.publish(normals_);
  }

  void Homework3::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Implement passthrough filter here. Put final output in the 'cloud_out' argument
    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud (cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cfg_.x_min, cfg_.x_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (cfg_.y_min, cfg_.y_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (cfg_.z_min, cfg_.z_max);
    pass.filter (*cloud_out);

 
  }

  void Homework3::voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Implement voxel downsampling filter here. Put final output in the 'cloud_out' argument
    // Run through a voxel grid filter to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_in);
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
    downsample.filter(*cloud_out);
  }

  void Homework3::normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(cloud_in);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(cloud_in);
    normal_estimator.setKSearch(kSearch_);
    normal_estimator.compute(*cloud_normals);

    // Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    for (int i = 0; i < cloud_normals->points.size(); i++) {
      // Compute the angle from vertical for the current normal vector
      //  and if it is greater than 30 degrees from vertical, add the index
      //  to the 'non_vertical_normals' array
      double Nx = cloud_normals->points.at(i).normal_x;
      double Ny = cloud_normals->points.at(i).normal_y;
      double Nz = cloud_normals->points.at(i).normal_z;

      double normal_angle = acos(abs(Nz))*(180/M_PI);

      if (normal_angle < 30){
        continue;
      }

      non_vertical_normals.indices.push_back(i);
    }

    pcl::copyPointCloud(*cloud_in, non_vertical_normals, *cloud_out);

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(cloud_in->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = cloud_in->points[non_vertical_normals.indices[i]].x;
      p.position.y = cloud_in->points[non_vertical_normals.indices[i]].y;
      p.position.z = cloud_in->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      // Construct rotation matrix to align frame transform with the normal vector
      tf2::Matrix3x3 rot_mat;
      // First basis vector is the vector we want to align
      rot_mat[0] = tf2::Vector3(nx, ny, nz);
      if (std::abs(nz) < 0.9) {
        // Vector is not close to vertical --> use x and y components to create orthogonal vector
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
      } else {
        // Vector is close to vertical --> use y and z components to make orthogonal vector
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
      }
      // Normalize the generated orthogonal vector, because it is not necessarily unit length
      rot_mat[1].normalize();
      // Cross product produces the third basis vector of the rotation matrix
      rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

      // Extract equivalent quaternion representation for the transform
      // rot_mat.transpose() is used because the basis vectors should be loaded
      // into the columns of the matrix, but the indexing in the above commands set the rows
      //   of the matrix instead of the columns.
      tf2::Quaternion q;
      rot_mat.transpose().getRotation(q);

      // Fill orientation of pose structure
      tf2::convert(q, p.orientation);
      normals_.poses.push_back(p);
    }
  }

  void Homework3::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // Implement Euclidean clustering here, dumping the array of separated clouds into the 'cluster_clouds' argument
    // Run Euclidean clustering and extract set of indices arrays
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tol);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(cloud_in);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    // Use indices arrays to separate point cloud into individual clouds for each cluster
    for (auto indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_in, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }
  }

  void Homework3::mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // Loop through the 'cluster_clouds' argument and merge all the points into a single PCL
    // point cloud. Then copy the merged cloud into the 'merged_cluster_cloud_' message
      pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& cluster : cluster_clouds) {
      merged_cloud->points.insert(merged_cloud->points.begin(), cluster->points.begin(), cluster->points.end());
    }
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;
    pcl::toROSMsg(*merged_cloud, merged_cluster_cloud_);
    
  }

  void Homework3::generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    pcl::PointXYZ min_point, max_point;
    bboxes_.objects.clear();
    int bbox_id = 0;
    for (auto& cluster : cluster_clouds) {
      pcl::getMinMax3D(*cluster, min_point, max_point);
      avs_lecture_msgs::TrackedObject box;
      box.header = bboxes_.header;
      // Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      box.id = bbox_id++;
      box.spawn_time = ros::Time::now();
      box.bounding_box_scale.x = max_point.x - min_point.x;
      box.bounding_box_scale.y = max_point.y - min_point.y;
      box.bounding_box_scale.z = max_point.z - min_point.z;
      box.pose.position.x = 0.5 * (max_point.x + min_point.x);
      box.pose.position.y = 0.5 * (max_point.y + min_point.y);
      box.pose.position.z = 0.5 * (max_point.z + min_point.z);
      box.pose.orientation.w = 1.0;
      bboxes_.objects.push_back(box);

    }
  }


}
