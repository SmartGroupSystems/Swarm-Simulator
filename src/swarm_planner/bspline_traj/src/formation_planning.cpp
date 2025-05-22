#include <bspline_race/formation_planning.h>


using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "formation_planning");
    ros::NodeHandle nh("~");
    
    ROS_INFO("\033[1;32m formation planner initialization complete.\033[0m");

    gvf_manager manager(nh);

    ros::AsyncSpinner spinner(8); //
    spinner.start();
    ros::waitForShutdown();
    
    return 0;
}

