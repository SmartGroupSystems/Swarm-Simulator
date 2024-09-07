#ifndef  _BSPLINE_RACE_H
#define  _BSPLINE_RACE_H

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
#include <ros/topic_manager.h>
#include <std_msgs/String.h>

//自定义
#include <bspline_race/UniformBspline.h>
#include <bspline_race/bspline_opt.h>
#include "common_msgs/common_msgs.h"
#include <plan_env/edt_environment.h>
#include <path_searching/astar.h>

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
            double lambda1_,lambda2_,lambda3_;
            double planInterval;
            std::mutex mtx;

            //智能类指针struct
            std::shared_ptr<UniformBspline> u;
            // std::vector<std::shared_ptr<UniformBspline>> swarm_bspline;
            // std::vector<std::shared_ptr<bspline_optimizer>> swarm_opt;
            // std::vector<std::shared_ptr<Astar>> swarm_astar;
            // std::vector<std::shared_ptr<SDFMap>> sdf_maps;
            // std::vector<std::shared_ptr<EDTEnvironment>> edt_environments;
            struct particleManager {
                std::string particle_index;
                std::shared_ptr<SDFMap> sdf_map_;
                std::shared_ptr<EDTEnvironment> edt_environment_;
                std::shared_ptr<Astar> geo_path_finder_;
                std::shared_ptr<bspline_optimizer> bspline_opt_;
                std::shared_ptr<UniformBspline> spline_;
            };
            std::vector<particleManager> swarmParticlesManager;

            
            //Traj
            common_msgs::BsplineTraj traj_;//执行轨迹

            //Particles
            bool isFirstCall = true; 
            common_msgs::Swarm_particles current_particles;
            common_msgs::Swarm_particles init_particles;
            common_msgs::Swarm_particles particles_goal;

        public:
            //ROS
            ros::Subscriber particles_sub; 
            ros::Publisher  traj_vis;
            ros::Publisher  traj_puber;
            ros::Publisher  waypoint_vis;  
            ros::Timer      traj_timer;
            ros::Subscriber goal_sub;
            ros::Publisher  path_vis;
            ros::Time       lastPlanTime;
            ros::Time       lastWaitOutputTime;

            //fsm
            bool receive_goal = false;
            bool exec_traj = false;
        public:
            plan_manager(){};  
            plan_manager(ros::NodeHandle& nh); 
            ~plan_manager();
            void testInit(ros::NodeHandle& nh); 
            void initCallback(ros::NodeHandle &nh);
            common_msgs::BsplineTraj getSmoothTraj(const std::vector<Point> waypoints);
            void optTraj();
            void parallelInit(ros::NodeHandle &nh);
            void update(const common_msgs::Swarm_particles& particles);
            void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
            void timerCallback(const ros::TimerEvent&);
            void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
            void visualizePath(const std::vector<Eigen::Vector3d>& path_points, ros::Publisher& marker_pub, const std::string& particle_index);
            void processParticle(size_t index, const common_msgs::Swarm_particles& init_particles, 
                                       const common_msgs::Swarm_particles& particles_goal, 
                        std::vector<particleManager>& swarmParticlesManager, 
                        ros::Publisher& path_vis, std::mutex& mtx); 
    };

}
#endif
