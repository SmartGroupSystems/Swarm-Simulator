#ifndef __NEIGHBORS_H__
#define __NEIGHBORS_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>

#include "common_msgs/Odom.h"
#include "common_msgs/OdomWithNeighbors.h"
#include "common_msgs/OdomBroadcast.h"

void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name);
std::map<std::string, ros::Subscriber> subscribers;
std::map<std::string, ros::Publisher>  odom_neighbors_publishers;
ros::Publisher odomBroadcast_pub;

double neighbor_dist_;
double threshold_dist_;

std::vector<common_msgs::Odom> broadcast_odom;
std::vector<std_msgs::String> uav_names;

// Function to extract the UAV number from the input string
inline std::string extractUavName(const std::string& uav_name) {
    std::regex pattern("/uav\\d+"); // Regular expression to match /uav followed by one or more digits
    std::smatch match;

    // Search for the pattern in the input string
    if (std::regex_search(uav_name, match, pattern)) {
        return match.str(0); // Return the matched string
    }
    return ""; // Return empty string if no match is found
}

#endif