#include "bspline_race/bspline_race.h"

namespace FLAG_Race

{
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        // testInit(nh);
        setParam(nh);
        parallelInitESDF(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::setParam(ros::NodeHandle &nh)
    {
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/TrajSampleRate", TrajSampleRate, -1);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        beta = max_vel_/0.5;

    }


    common_msgs::BsplineTraj plan_manager::getSmoothTraj(const std::vector<Point> waypoints)
    {
        common_msgs::BsplineTraj traj;
        std::vector<Eigen::Vector2d> control_points_;
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        initial_state <<        waypoints.front().x, waypoints.front().y,
                                0.0,                 0.0,
                                0.0,                 0.0;
        terminal_state<<        waypoints.back().x,  waypoints.back().y,
                                0.0,                 0.0,
                                0.0,                 0.0;
        A_ini <<                1.0/6,              2.0/3,                  1.0/6,
                                -1.0/2*beta,        0.0*beta,               1.0/2*beta,
                                1.0*beta*beta,      -2.0*beta*beta,         1.0*beta*beta;
        A_ter<<                 1.0/6,              2.0/3,                  1.0/6,
                                -1.0/2*beta,        0.0*beta,               1.0/2*beta,
                                1.0*beta*beta,      -2.0*beta*beta,         1.0*beta*beta;

        Eigen::MatrixXd tmp1(3,2);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,2);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(initial_state);
        tmp2 = A_ter.colPivHouseholderQr().solve(terminal_state);

        // 将 tmp1 的值添加到 control_points_
        for (int i = 0; i < tmp1.rows(); ++i) {
            control_points_.push_back(Eigen::Vector2d(tmp1(i, 0), tmp1(i, 1)));
        }

        // 处理 waypoints
        Eigen::Vector2d last_point = control_points_.back();
        for (size_t i = 1; i < waypoints.size(); ++i) {
            Eigen::Vector2d current_point(waypoints[i].x, waypoints[i].y);
            double distance = (current_point - last_point).norm();

            if (distance < 0.5) {
                // 如果当前 waypoint 与 last_point 的距离小于0.5，直接存储
                control_points_.push_back(current_point);
            } else {
                // 否则，计算新的控制点
                Eigen::Vector2d direction = (current_point - last_point).normalized();
                Eigen::Vector2d new_control_point = last_point + 0.5 * direction;
                control_points_.push_back(new_control_point);
                i--; // 保持在当前 waypoint 上，以便再次检查
            }

            last_point = control_points_.back(); // 更新最后一个点
        }

        // 将 tmp2 的值添加到 control_points_
        for (int i = 0; i < tmp2.rows(); ++i) {
            control_points_.push_back(Eigen::Vector2d(tmp2(i, 0), tmp2(i, 1)));
        }

        // 创建一个大小适当的 Eigen::MatrixXd 矩阵
        Eigen::MatrixXd control_points_matrix(control_points_.size(), 2);

        // 将数据从 vector 复制到矩阵
        for (size_t i = 0; i < control_points_.size(); ++i) {
            control_points_matrix(i, 0) = control_points_[i](0); // x 坐标
            control_points_matrix(i, 1) = control_points_[i](1); // y 坐标
        } 

        cps_num = control_points_.size();
        u.reset(new UniformBspline(p_order_,cps_num,beta,Dim_,initial_state,terminal_state));
        u->setControlPoints(control_points_matrix);
        u->getT(TrajSampleRate);
        UniformBspline p = *u;
        UniformBspline v = p.getDerivative();
        UniformBspline a = v.getDerivative();
        UniformBspline j = a.getDerivative();
        p_ = p.getTrajectory(p.time_);
        v_ = v.getTrajectory(p.time_);
        a_ = a.getTrajectory(p.time_);
        j_ = j.getTrajectory(p.time_);

        traj.position.clear();
        traj.velocity.clear();
        traj.acceleration.clear();

        geometry_msgs::PoseStamped tmp_p,tmp_v,tmp_a,tmp_j;
        //traj : px py vx vy ax ay
        for (size_t i = 0; i < p_.rows(); i++)
        {    
            tmp_p.pose.position.x   = p_(i,0);tmp_p.pose.position.y   = p_(i,1); tmp_p.pose.position.z   = 0;
            tmp_v.pose.position.x   = v_(i,0); tmp_v.pose.position.y  = v_(i,1); tmp_v.pose.position.z   = 0;
            tmp_a.pose.position.x   = a_(i,0); tmp_a.pose.position.y  = a_(i,1); tmp_a.pose.position.z   = 0; 
            tmp_j.pose.position.x   = j_(i,0); tmp_j.pose.position.y  = j_(i,1); tmp_j.pose.position.z   = 0;
            traj.position.push_back(tmp_p) ;
            traj.velocity.push_back(tmp_v) ;
            traj.acceleration.push_back(tmp_a);
            traj.jerk.push_back(tmp_j);
        } 
        traj.header.frame_id = "world";
        traj.header.stamp = ros::Time::now();  
        traj.header.seq = 1;
        return traj;
    }

    void plan_manager::update(const common_msgs::Swarm_particles& particles) 
    {
        current_particles = particles;  
    }

    void plan_manager::optTraj()
    {
        
    }

    void plan_manager::parallelInitESDF(ros::NodeHandle &nh) {
        ros::master::V_TopicInfo topic_list;
        ros::master::getTopics(topic_list);
        std::regex topic_pattern("/particle(\\d+)/odom");
        std::smatch match;
        
        for (const auto &info : topic_list) {
            if (info.datatype == "nav_msgs/Odometry" && std::regex_search(info.name, match, topic_pattern)) {
                try {
                    std::string particle_index = match[1];
                    std::string particle_base = "/particle" + particle_index;
                    std::string odom_topic = particle_base + "/odom";
                    std::string cloud_topic = "/map_generator/global_cloud";

                    // 创建 SDFMap 的 shared_ptr 实例，并初始化
                    auto sdf_map_ = std::make_shared<SDFMap>();
                    sdf_map_->initMap(nh, particle_base, odom_topic, cloud_topic);
                    auto edt_environment_ = std::make_shared<EDTEnvironment>();
                    edt_environment_->setMap(sdf_map_);
                    sdf_maps.push_back(sdf_map_);
                    edt_environments.push_back(edt_environment_);

                } catch (const std::exception& e) {
                    ROS_ERROR("Exception caught while initializing environments for %s: %s", info.name.c_str(), e.what());
                } catch (...) {
                    ROS_ERROR("Unknown exception caught during initialization for %s", info.name.c_str());
                }
            }
        }
    }

    void plan_manager::testInit(ros::NodeHandle& nh)
    {
        ros::NodeHandle node_;
        node_ = nh;
        ROS_INFO("Test Init~~~~~!");
    }

}