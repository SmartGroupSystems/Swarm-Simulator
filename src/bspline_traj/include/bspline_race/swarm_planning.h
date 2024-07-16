#ifndef  _SWARM_PLANNING_H
#define  _SWARM_PLANNING_H   

#include <bspline_race/bspline_race.h>
#include "common_msgs/common_msgs.h"

using namespace std;

ros::Publisher traj_vis;
ros::Publisher traj_puber;
ros::Publisher waypoint_vis;
ros::Subscriber particles_sub;   
ros::Timer      traj_timer;
std::map<std::string, ros::Subscriber> odom_subscribers;

common_msgs::Swarm_particles latest_swarm_particles;


void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name);

#endif