#include <bspline_race/swarm_planning.h>


using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "swarm_planning");
    ros::NodeHandle nh("~");
    nh.param("fsm/sphInitTime", sphInitTime_, 5.0);

    ros::Duration(sphInitTime_).sleep();
    
    plan_manager manager(nh);

    ros::AsyncSpinner spinner(8); // Use 8 threads
    spinner.start();
    ros::waitForShutdown();

    // ros::spin();
    return 0;
}

