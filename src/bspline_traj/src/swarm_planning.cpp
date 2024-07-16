#include <bspline_race/swarm_planning.h>


using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "swarm_planning");
    ros::NodeHandle nh("~");
    ros::Duration(3).sleep();
 
    traj_puber = nh.advertise<common_msgs::Swarm_traj>("/swarm_traj", 10, true);
    particles_sub = nh.subscribe("/swarm_particles", 1000, particlesCallback);
    traj_timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    
    plan_manager manager(nh);

    ros::spin();
    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name) 
{

}

void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh) 
{
    odom_subscribers[topic_name] = nh.subscribe<nav_msgs::Odometry>(
        topic_name, 
        1000, 
        boost::bind(odomCallback, _1, topic_name)
    );
}

void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    // manager.update(*msg); 
    latest_swarm_particles = *msg;
}

void timerCallback(const ros::TimerEvent&) 
{
    common_msgs::Swarm_traj new_traj;
    // 填充 new_traj 的数据

    traj_puber.publish(new_traj);
}