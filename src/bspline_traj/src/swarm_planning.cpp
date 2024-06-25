#include <bspline_race/swarm_planning.h>

using namespace FLAG_Race;

ros::Publisher traj_vis;
ros::Publisher traj_puber;
ros::Publisher waypoint_vis;

ros::Subscriber particles_sub;   
    
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "swarm_planning");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);

    waypoint_vis = nh.advertise<visualization_msgs::Marker>("/waypoint_vis", 10, true);
    traj_puber = nh.advertise<bspline_race::BsplineTraj>("/swarm_traj", 10, true);
    traj_vis = nh.advertise<visualization_msgs::Marker>("/traj_vis", 10, true);
    particles_sub = nh.subscribe("particles_vis", 1000, particlesCallback);


    ros::spin();
    return 0;
}

void particlesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    ROS_INFO("Received MarkerArray with %lu markers", msg->markers.size());


}