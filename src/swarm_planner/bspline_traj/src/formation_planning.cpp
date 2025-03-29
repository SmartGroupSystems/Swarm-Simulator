#include <bspline_race/formation_planning.h>


using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "formation_planning");
    ros::NodeHandle nh("~");
    nh.param("fsm/sphInitTime", sphInitTime_, 5.0);

    ros::Duration(sphInitTime_).sleep();
    
    ROS_INFO("\033[1;32m formation planner initialization complete.\033[0m");

    gvf_manager manager(nh);

    ros::AsyncSpinner spinner(15); // Use 15 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

