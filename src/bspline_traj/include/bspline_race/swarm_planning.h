#ifndef  _SWARM_PLANNING_H
#define  _SWARM_PLANNING_H   

#include <bspline_race/bspline_race.h>
#include "common_msgs/common_msgs.h"

using namespace std;

ros::Publisher traj_vis;
ros::Publisher traj_puber;
ros::Publisher waypoint_vis;
ros::Subscriber particles_sub;   

common_msgs::Swarm_particles latest_swarm_particles;

void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg);

#endif