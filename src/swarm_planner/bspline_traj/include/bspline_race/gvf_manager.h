#ifndef  _GVF_MANAGER_H
#define  _GVF_MANAGER_H   

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
#include "bspline_race/gvf.h"

using namespace std;

namespace FLAG_Race
{

class gvf_manager
{
    public:
        double init_bias_x, init_bias_y;
        double planInterval;
        std::string cloud_topic_, odom_topic_;
        Eigen::Vector3d odom_;

        struct gvfManager {
            std::string index;
            std::shared_ptr<SDFMap> sdf_map_;
            std::shared_ptr<EDTEnvironment> edt_environment_;
            std::shared_ptr<Astar> geo_path_finder_;
            std::shared_ptr<KinodynamicAstar> kino_path_finder_;
            std::shared_ptr<gvf>  gvf_;
            ros::Time curr_time;  // 当前时间定时器
            ros::Time last_time;  // 上一次时间定时器
            bool is_initialized = false; 
            bool receive_startpt = false;
            // ros::Subscriber odom_sub;
            Eigen::Vector3d start_pt, goal_pt, odom;
        };

        std::vector<gvfManager> swarmParticlesManager;

    public:
        //ROS
        ros::Publisher  force_pub;
        ros::Timer      exec_timer;
        ros::Publisher  cmd_pub;
        ros::Timer      cmd_timer;
        ros::Subscriber goal_sub;
        ros::Publisher  path_vis;
        ros::Subscriber odom_sub;
        ros::Publisher  path_pub; 

    public:
        gvf_manager(){};  
        gvf_manager(ros::NodeHandle& nh); 
        ~gvf_manager();
        void initCallback(ros::NodeHandle &nh);
        void InitGvf(ros::NodeHandle &nh);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
        void AstarExecCallback(const ros::TimerEvent& event);
        void cmdCallback(const ros::TimerEvent& event);
        void publishCorridorMarker(double C_thresh = -1.0);
        void visualizePath(const std::vector<Eigen::Vector3d>& path_points, 
                            ros::Publisher& marker_pub, const std::string& particle_index);
        std::vector<Eigen::Vector3d> correctPathToCenter(
                const std::vector<Eigen::Vector3d>& raw_path);


        //inline func 
        inline Eigen::Vector3d esdfGrad(const Eigen::Vector3d& p) const
        {
            Eigen::Vector3d grad;
            swarmParticlesManager[0]
                .sdf_map_->getDistWithGradTrilinear(p, grad);   // 距离返回值可忽略
            return grad;                                        // 单位: m
        }
};

}

#endif