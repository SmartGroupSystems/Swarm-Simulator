#include <bspline_race/swarm_planning.h>

using namespace FLAG_Race;

plan_manager manager;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "swarm_planning");
    ros::NodeHandle nh("~");

    manager = plan_manager(nh);
      
    traj_puber = nh.advertise<common_msgs::Swarm_traj>("/swarm_traj", 10, true);
    particles_sub = nh.subscribe("/swarm_particles", 1000, particlesCallback);
    traj_timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    
    ros::spin();
    return 0;
}

void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    manager.update(*msg); 
    // latest_swarm_particles = *msg;
}

void timerCallback(const ros::TimerEvent&) 
{
    common_msgs::Swarm_traj new_traj;
    // 填充 new_traj 的数据

    traj_puber.publish(new_traj);
}