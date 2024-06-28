#include <bspline_race/swarm_planning.h>

using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "swarm_planning");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);
      
    traj_puber = nh.advertise<common_msgs::Swarm_traj>("/swarm_traj", 10, true);
    particles_sub = nh.subscribe("/swarm_particles", 1000, particlesCallback);

    ros::spin();
    return 0;
}

void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    latest_swarm_particles = *msg;
    // ROS_INFO("Received new swarm particles data with %ld particles", msg->particles.size());
}