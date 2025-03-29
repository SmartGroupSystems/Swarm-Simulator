#ifndef  _GVF_H
#define  _GVF_H   

//standard
#include <fstream>
#include <string>
#include <regex>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <numeric>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>

//自定义
#include <bspline_race/UniformBspline.h>
#include <bspline_race/bspline_opt.h>
#include "common_msgs/common_msgs.h"
#include <plan_env/edt_environment.h>
#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>

using namespace std;


namespace FLAG_Race
{
    class gvf_manager
    {
        public:
           
        public:
            gvf_manager(){};  
            gvf_manager(ros::NodeHandle& nh); 
            ~gvf_manager();

    };

}

#endif