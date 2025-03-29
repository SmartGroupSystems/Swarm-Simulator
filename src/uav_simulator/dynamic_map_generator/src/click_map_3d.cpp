#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

ros::Publisher all_map_pub_;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_;

ros::Subscriber click_sub_;
vector<Eigen::Vector3d> points_;
double len2_;

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double z = msg.pose.position.z;

  pcl::PointXYZ pt_random;

  static std::default_random_engine generator(std::random_device{}());
  std::uniform_real_distribution<double> distribution(0.0, 2.0);
  double random_offset = distribution(generator);

  // 在 (x, y) 为中心，以 len2_ + random_offset 为边长的正方形区域生成点云
  double half_len = (len2_ + random_offset) / 2.0;

  for (double dx = -half_len; dx <= half_len; dx += 0.3) {
    for (double dy = -half_len; dy <= half_len; dy += 0.3) {
      for (double dz = -0.1; dz <= z; dz += 0.1) {
        pt_random.x = x + dx;
        pt_random.y = y + dy;
        pt_random.z = dz;
        map_cloud_.push_back(pt_random);
      }
    }
  }


  map_cloud_.width = map_cloud_.points.size();
  map_cloud_.height = 1;
  map_cloud_.is_dense = true;
  pcl::toROSMsg(map_cloud_, map_msg_);
  map_msg_.header.frame_id = "world";
  all_map_pub_.publish(map_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_map");
  ros::NodeHandle n("~");

  n.param("map/len2", len2_, 1.0);

  all_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/click_map", 1);

  click_sub_ = n.subscribe("/move_base_simple/goal", 10, clickCallback);

  ros::Duration(0.5).sleep();

  // init random device

  while (ros::ok()) {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = "world";
    all_map_pub_.publish(map_msg_);
    ROS_INFO("Start building map, use 2d_nav_goal click the map, then a pointcloud would generate.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}