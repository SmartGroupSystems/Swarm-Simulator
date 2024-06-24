#ifndef  _BSPLINE_RACE_H
#define  _BSPLINE_RACE_H

//standard
#include <fstream>
#include <string>
#include <vector>

//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

//自定义
#include <bspline_race/UniformBspline.h>
#include <bspline_race/bspline_opt.h>
#include <bspline_race/BsplineTraj.h>
#include <bspline_race/PositionCommand.h>

using namespace std;

struct Point { double x, y, z;};


namespace FLAG_Race
{
    class plan_manager
    {
        public:
            //从launch读取的参数

            int p_order_;// order of bspline
            int N_;// number of control points
            int Dim_;// dimension of traj
            int TrajSampleRate;// 轨迹采样频率
            int cps_num;  
            double beta;
            double max_vel_,max_acc_;//最大速度，加速度
            Eigen::MatrixXd initial_state,terminal_state;//初始，结束P V A
            Eigen::MatrixXd p_,v_,a_,j_;//轨迹buffer
            Eigen::MatrixXd A_ini, A_ter;

            //智能类指针
            std::shared_ptr<UniformBspline> u;
            double lambda1_,lambda2_;
            
            //Traj
            bspline_race::BsplineTraj traj_;//执行轨迹

        public:
            plan_manager(){}
            plan_manager(ros::NodeHandle &nh);
            ~plan_manager();
            void setParam(ros::NodeHandle &nh);//从ros节点中读取参数
            bspline_race::BsplineTraj getSmoothTraj(const std::vector<Point> waypoints);
            void optTraj();
    };

}
#endif
