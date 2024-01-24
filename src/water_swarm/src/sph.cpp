#include "sph.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_planner");
    ros::NodeHandle nh("~");

    ROS_INFO("sph_planner node has started.");
    

    ros::spin();
    return 0;
}