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
#include <path_searching/astar_topo.h>
#include <path_searching/kinodynamic_astar.h>
#include "bspline_race/gvf.h"

using namespace std;

#define PI acos(-1)
#define INF 999.9
double delta_T = 0.02;
double last_yaw;
double last_yaw_dot;
double roll, pitch, yaw;//定义存储r\p\y的容器
double D_YAW_MAX = PI/2;
double output_yaw;
double output_d_yaw;
double YAW_MAX = D_YAW_MAX * delta_T;

namespace FLAG_Race
{

class gvf_manager
{
    public:
        double init_bias_x, init_bias_y;
        double planInterval;
        std::string cloud_topic_, odom_topic_;
        Eigen::Vector3d odom_;
        bool use_kinopath_ ;

        struct gvfManager {
            std::string index;
            std::shared_ptr<SDFMap> sdf_map_;
            std::shared_ptr<EDTEnvironment> edt_environment_;
            std::shared_ptr<AstarTopo> geo_path_finder_;
            std::shared_ptr<KinodynamicAstar> kino_path_finder_;
            std::shared_ptr<bspline_optimizer> bspline_opt_;
            std::shared_ptr<UniformBspline> spline_;
            std::shared_ptr<gvf>  gvf_;
            ros::Time curr_time;  // 当前时间定时器
            ros::Time last_time;  // 上一次时间定时器
            bool is_initialized = false; 
            bool receive_startpt = false;
            bool is_first_goal = true;  // 添加标志位
            bool is_first_kinogoal = true;
            std::vector<Eigen::Vector3d> last_path;  // 存储上一次的轨迹
            Eigen::MatrixXd last_traj;  // 存储上一次的轨迹矩阵
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
        ros::Publisher  kino_path_pub;  // 新增发布者
        ros::Timer      kino_timer;     // 新增定时器
        ros::Publisher  goal_vis_pub;   // 新增目标点可视化发布者

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
        void KinoPathCallback(const ros::TimerEvent& event);  // 新增回调函数
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

    inline std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw)
    {
    std::pair<double, double> yaw_yawdot(0, 0);
    if(current_yaw<0)                 current_yaw = current_yaw + 2*PI;
    else if(current_yaw>2*PI)  current_yaw = current_yaw - 2*PI;
        if(aim_yaw<0)                 aim_yaw = aim_yaw + 2*PI;
    else if(aim_yaw>2*PI)    aim_yaw = aim_yaw - 2*PI;
    double yaw_distance = aim_yaw - current_yaw;
    double sign_        = yaw_distance / fabs(yaw_distance);
    if(fabs(yaw_distance) < YAW_MAX )
    {cout<<"ca1"<<endl;
        output_yaw   = aim_yaw;
        output_d_yaw = yaw_distance / delta_T;
    }
    else
    {cout<<"ca2"<<endl;
        output_yaw = current_yaw + sign_ * YAW_MAX;
        output_d_yaw = sign_*D_YAW_MAX;
    }
    yaw_yawdot.first = output_yaw;
    yaw_yawdot.second = output_d_yaw;
    return yaw_yawdot;
    }
    
    inline std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw)
    {
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw_ = 0;
    double yawdot = 0;
    if (aim_yaw - current_yaw > PI)
    {
        
        if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
        {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
            yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
            yawdot = -D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
    else if (aim_yaw - current_yaw < -PI)
    {
        if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
        {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
            yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw < -PI)
            yawdot = D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
    else
    {
        if (aim_yaw - current_yaw < -YAW_MAX)
        {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
            yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
        }
        else if (aim_yaw - current_yaw > YAW_MAX)
        {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
            yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
            yawdot = -D_YAW_MAX;
        else if (yaw_ - current_yaw < -PI)
            yawdot = D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
        if (fabs(yaw_ - last_yaw) <= YAW_MAX)
        yaw = 0.5 * last_yaw + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
    last_yaw = yaw_;  
    last_yaw_dot = yawdot;
    yaw_yawdot.first = yaw_;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
    }
};

}

#endif