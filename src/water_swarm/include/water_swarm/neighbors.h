#ifndef __NEIGHBORS_H__
#define __NEIGHBORS_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <regex>
#include <map>

void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name);
std::map<std::string, ros::Subscriber> subscribers;

double neighbor_dist_;

struct Position {
    double x, y, z;
};

struct Velocity {
    double x, y, z;
};

struct Odom {
    Position position;
    Velocity velocity;
};


#endif