#include <gtest/gtest.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>
#include <geometry_msgs/PoseArray.h>
#include <rosgraph_msgs/Clock.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseArray, avs_lecture_msgs::TrackedObjectArray> TestSyncPolicy;

ros::NodeHandle* node;

ros::Subscriber sub_cluster_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud;
bool received_cluster_cloud;

ros::Subscriber sub_normals;
geometry_msgs::PoseArray normals;
bool received_normals;

ros::Subscriber sub_objects;
avs_lecture_msgs::TrackedObjectArray objects;
bool received_objects;

bool received_synced_topics;
bool using_sim_time;
bool received_clock;

ros::WallTime last_clock_tick;

static constexpr double G_TIME_RESOLUTION = 0.01;
static constexpr double G_TOPIC_TIMEOUT = 10.0;
static constexpr double G_TEST_DURATION = 60.0;

bool foundMessages()
{
  return received_cluster_cloud && received_normals && received_objects && received_clock && received_synced_topics;
}

void runCheck()
{
  ros::WallTime timeout_t = ros::WallTime::now() + ros::WallDuration(G_TEST_DURATION);
  while (!ros::isShuttingDown() && ((timeout_t - ros::WallTime::now()).toSec() > 0)) {
    ros::WallDuration time_since_bag_clock = ros::WallTime::now() - last_clock_tick;
    if (time_since_bag_clock > ros::WallDuration(0.04)) {
      return;
    }

    if (received_synced_topics) {
      received_synced_topics = false;

      ASSERT_STREQ(cluster_cloud->header.frame_id.c_str(), "base_footprint") << "Cluster cloud frame ID not correct";
      ASSERT_STREQ(normals.header.frame_id.c_str(), "base_footprint") << "Normals frame ID not correct";
      ASSERT_STREQ(objects.header.frame_id.c_str(), "base_footprint") << "Objects frame ID not correct";

      ASSERT_GE(objects.objects.size(), 3) << "Not enough clusters detected";
      ASSERT_GE(cluster_cloud->points.size(), 200) << "Not enough points in cluster cloud";
      ASSERT_GE(normals.poses.size(), 200) << "Not enough poses in normals array";

      for (size_t i = 0; i < objects.objects.size(); i++) {
        avs_lecture_msgs::TrackedObject& obj = objects.objects[i];
        ASSERT_TRUE(obj.bounding_box_scale.x > 0 && obj.bounding_box_scale.y > 0 && obj.bounding_box_scale.z > 0) << "Object message contained a bounding box with zero size";
        ASSERT_TRUE(obj.bounding_box_scale.x < 6.0 || obj.bounding_box_scale.y < 6.0) << "Object message contained a bounding box with too much planar area (" << obj.bounding_box_scale.x << "m x " << obj.bounding_box_scale.y << "m)";
        ASSERT_TRUE(obj.pose.position.x != 0.0 || obj.pose.position.y != 0.0) << "Object message contained a bounding box with zero position";
        ASSERT_EQ(obj.id, (uint32_t)i) << "Incorrect bounding box ID detected";
        ASSERT_DOUBLE_EQ(obj.pose.orientation.w, 1.0) << "Object message contained a bounding box with an invalid quaternion (should be w = 1, x,y,z = 0)";
      }

      for (auto n : normals.poses) {
        double val = 2 * (n.orientation.x * n.orientation.z - n.orientation.w * n.orientation.y); // Z component of first basis vector
        double ang = acos(val);

        double up_thres = M_PI / 6 - 0.01;
        double down_thres = 5.0 * M_PI / 6.0 + 0.01;
        ASSERT_TRUE((ang > up_thres) && (ang < down_thres)) << "Detected a normal vector at incorrect angle: " << (180.0 / M_PI * ang) << " degrees";
      }
    }

    ros::spinOnce();
    ros::WallDuration(G_TIME_RESOLUTION).sleep();
  }
}

void recvClusterCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  received_cluster_cloud = true;
  sub_cluster_cloud.shutdown();
}

void recvNormals(const geometry_msgs::PoseArrayConstPtr& msg)
{
  received_normals = true;
  sub_normals.shutdown();
}

void recvObjects(const avs_lecture_msgs::TrackedObjectArray& msg)
{
  received_objects = true;
  sub_objects.shutdown();
}

void recvSyncedTopics(const sensor_msgs::PointCloud2ConstPtr& cluster_cloud_msg,
                      const geometry_msgs::PoseArrayConstPtr& normals_msg,
                      const avs_lecture_msgs::TrackedObjectArrayConstPtr& objects_msg)
{
  normals = *normals_msg;
  objects = *objects_msg;
  pcl::fromROSMsg(*cluster_cloud_msg, *cluster_cloud);
  received_synced_topics = true;
}

TEST(Homework3, test_point_cloud_processing)
{
  ASSERT_TRUE(using_sim_time) << "ROS is not using the clock from the bag file";
  ros::WallTime timeout_t = ros::WallTime::now() + ros::WallDuration(G_TOPIC_TIMEOUT);
  while(!ros::isShuttingDown() && !foundMessages() && ((timeout_t - ros::WallTime::now()).toSec() > 0.0)) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  EXPECT_TRUE(received_clock) << "No clock being published";
  EXPECT_TRUE(received_cluster_cloud) << "Test did not receive cluster cloud message";
  EXPECT_TRUE(received_normals) << "Test did not receive normals message";
  EXPECT_TRUE(received_objects) << "Test did not receive object message";
  EXPECT_TRUE(received_synced_topics) << "Test did not successfully synchronize input messages";
  ASSERT_TRUE(foundMessages()) << "Missing messages";
  runCheck();
}

void recvClock(const rosgraph_msgs::ClockConstPtr& msg)
{
  last_clock_tick = ros::WallTime::now();
  received_clock = true;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "homework3_tests");

  node = new ros::NodeHandle();

  sub_cluster_cloud = node->subscribe("/merged_cluster_cloud", 1, recvClusterCloud);
  sub_normals = node->subscribe("/normals", 1, recvNormals);
  sub_objects = node->subscribe("/objects", 1, recvObjects);
  ros::Subscriber sub_clock = node->subscribe("/clock", 1, recvClock);

  node->param("/use_sim_time", using_sim_time, false);

  message_filters::Subscriber<sensor_msgs::PointCloud2> sync_sub_cluster_cloud(*node, "/merged_cluster_cloud", 10);
  message_filters::Subscriber<geometry_msgs::PoseArray> sync_sub_normals(*node, "/normals", 10);
  message_filters::Subscriber<avs_lecture_msgs::TrackedObjectArray> sync_sub_objects(*node, "/objects", 10);
  message_filters::Synchronizer<TestSyncPolicy> sync(TestSyncPolicy(100), sync_sub_cluster_cloud, sync_sub_normals, sync_sub_objects);
  sync.registerCallback(boost::bind(&recvSyncedTopics, _1, _2, _3));

  received_cluster_cloud = false;
  received_normals = false;
  received_objects = false;
  received_synced_topics = false;
  cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  node->shutdown();
  delete node;

  return 0;
}
