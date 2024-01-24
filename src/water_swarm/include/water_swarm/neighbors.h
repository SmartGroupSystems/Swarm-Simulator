#ifndef __NEIGHBORS_H__
#define __NEIGHBORS_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>

#include "water_swarm/Odom.h"
#include "water_swarm/OdomWithNeighbors.h"
#include "water_swarm/OdomBroadcast.h"

void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name);
std::map<std::string, ros::Subscriber> subscribers;
std::map<std::string, ros::Publisher>  odom_neighbors_publishers;
ros::Publisher odomBroadcast_pub;

double neighbor_dist_;
double threshold_dist_;

std::vector<water_swarm::Odom> broadcast_odom;

#endif