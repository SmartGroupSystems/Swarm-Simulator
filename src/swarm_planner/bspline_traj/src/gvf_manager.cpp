#include "bspline_race/gvf_manager.h"

namespace FLAG_Race
{
    gvf_manager::gvf_manager(ros::NodeHandle &nh)
    {
        nh.param("gvf/planInterval", planInterval, -1.0);
        nh.param<std::string>("gvf/cloud_topic", cloud_topic_, "click_map");
        nh.param<std::string>("gvf/odom_topic", odom_topic_, "odom");
        nh.param("gvf/init_bias_x", init_bias_x, -1.0);
        nh.param("gvf/init_bias_y", init_bias_y, -1.0);
        nh.param("gvf/gvf_use_kinopath", use_kinopath_, false);
        initCallback(nh);
        InitGvf(nh);
    }
    gvf_manager::~gvf_manager() {}

    void gvf_manager::initCallback(ros::NodeHandle &nh)
    {
        exec_timer = nh.createTimer(ros::Duration(0.2), &gvf_manager::AstarExecCallback, this);
        force_pub  = nh.advertise<common_msgs::Swarm_particles>("/gvf_force", 10);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &gvf_manager::goalCallback, this);
        path_vis = nh.advertise<visualization_msgs::Marker>("/path_vis", 10);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &gvf_manager::odomCallback, this);
        cmd_pub    = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
        cmd_timer  = nh.createTimer(ros::Duration(0.02), &gvf_manager::cmdCallback, this);  // 50Hz
        path_pub = nh.advertise<nav_msgs::Path>("/particle0/path", 1);
        kino_path_pub = nh.advertise<nav_msgs::Path>("/particle0/kinopath", 1);  // 初始化新发布者
        kino_timer = nh.createTimer(ros::Duration(0.2), &gvf_manager::KinoPathCallback, this);  // 初始化新定时器
        goal_vis_pub = nh.advertise<visualization_msgs::Marker>("/goal_vis", 10);  // 初始化目标点可视化发布者
    } 

void gvf_manager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("[GVF] receive goal (%.2f, %.2f, %.2f)",
             msg->pose.position.x,
             msg->pose.position.y,
             msg->pose.position.z);

    // 发布目标点可视化
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_visualization";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置目标点的位置
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z+1.0;
    
    // 设置目标点的方向
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    
    // 设置目标点的尺寸
    marker.scale.x = 0.2;  // 球体直径
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    // 设置目标点的颜色为红色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // 不透明
    
    // 发布标记
    goal_vis_pub.publish(marker);

    Eigen::Vector3d start_pt(odom_.x()+0.000001, odom_.y()+0.000001,1.0);
    Eigen::Vector3d goal_pt(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z+1.0
    );

    for (auto& manager : swarmParticlesManager) {
        manager.receive_startpt = true;
        manager.start_pt = start_pt;
        manager.goal_pt = goal_pt;
        manager.is_first_goal = true;  // 重置标志位
    }
}

void gvf_manager::cmdCallback(const ros::TimerEvent& event)
{
    if (swarmParticlesManager.empty()) return;

    // 使用第一个粒子的位置信息
    const Eigen::Vector3d& pos = odom_;
    
    if (!swarmParticlesManager[0].receive_startpt) {
        // ROS_WARN_THROTTLE(2.0, "[gvf_manager] GVF not initialized.");
        return;
    }

    // 计算 GVF 速度
    Eigen::Vector3d vel = swarmParticlesManager[0].gvf_->calcGuidingVectorField3D(pos);

    // 构造 PositionCommand 消息
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";

    // 禁用 position 控制
    cmd.position.x = std::numeric_limits<float>::quiet_NaN();
    cmd.position.y = std::numeric_limits<float>::quiet_NaN();
    cmd.position.z = std::numeric_limits<float>::quiet_NaN();

    // 设置速度控制
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();

    double arg_    = atan2(-cmd.velocity.x,cmd.velocity.y) + (PI/2.0f);
    double vel_len = sqrt(pow(cmd.velocity.x,2)+pow(cmd.velocity.y,2));
    if(vel_len<=0.1) arg_ = last_yaw;
    std::pair<double, double> yaw_all = calculate_yaw(last_yaw,arg_);

    cmd.yaw = yaw_all.first;
    cmd.yaw_dot = yaw_all.second;
// ROS_INFO("[GVF] Velocity Command: x = %.3f, y = %.3f, z = %.3f",
//          cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    // 发布控制指令
    cmd_pub.publish(cmd);
}

    void gvf_manager::InitGvf(ros::NodeHandle &nh)
    {
        try {
            std::string particle_base = "/particle0";
            // //EDT & MAP
            auto sdf_map_ = std::make_shared<SDFMap>();
            sdf_map_->initMap(nh, particle_base, odom_topic_, cloud_topic_);
            auto edt_environment_ = std::make_shared<EDTEnvironment>();
            edt_environment_->setMap(sdf_map_);
            
            //ASTAR
            auto geo_path_finder_ = std::make_shared<AstarTopo>();
            geo_path_finder_->setParam(nh);
            geo_path_finder_->setEnvironment(edt_environment_);
            geo_path_finder_->init();

            // dynamic a*
            auto kino_path_finder_ = std::make_shared<KinodynamicAstar>();
            // kino_path_finder_->setParam(nh);
            // kino_path_finder_->setEnvironment(edt_environment_);
            // kino_path_finder_->init();

            //OPT
            auto bspline_opt_ = std::make_shared<bspline_optimizer>();
            bspline_opt_->init(nh);
            bspline_opt_->setEnvironment(edt_environment_);

            //UNIFORM BSPLINE
            auto spline_ = std::make_shared<UniformBspline>();
            spline_->init(nh);

            // gvf
            auto gvf_ = std::make_shared<gvf>();
            gvf_->init(nh, particle_base, odom_topic_, cloud_topic_);

            gvfManager pm {
                particle_base,
                sdf_map_,
                edt_environment_,
                geo_path_finder_,
                kino_path_finder_,
                bspline_opt_,
                spline_,
                gvf_,
                ros::Time::now(),     // curr_time
                ros::Time(0),         // last_time
                true, 
            };

            swarmParticlesManager.push_back(pm);  // 将实例存入向量

            std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught while initializing environments for %s", e.what());
        }  
    }

void gvf_manager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->odom_ = Eigen::Vector3d(
    msg->pose.pose.position.x+0.000001,
    msg->pose.pose.position.y+0.000001,
    msg->pose.pose.position.z);
}

void gvf_manager::visualizePath(const std::vector<Eigen::Vector3d>& path_points, 
                    ros::Publisher& marker_pub, const std::string& particle_index) 
{
    // 定义一个Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 根据实际使用的坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "path_visualization";
    
    // 使用 particle_index 作为 marker 的唯一 ID
    std::hash<std::string> hash_fn;
    marker.id = static_cast<int>(hash_fn(particle_index));
    
    // 使用POINTS类型以便可以设置每个点的颜色
    marker.type = visualization_msgs::Marker::POINTS; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; 
    marker.scale.y = 0.1;

    // 遍历路径点并将其添加到Marker中
    for (size_t i = 0; i < path_points.size(); ++i) {
        geometry_msgs::Point p;
        p.x = path_points[i].x();
        p.y = path_points[i].y();
        p.z = path_points[i].z();

        // 将当前点添加到Marker
        marker.points.push_back(p);

        // 设置每个点的颜色
        std_msgs::ColorRGBA color;
        if (i == path_points.size() - 1) {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;

        } else {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
        }
        // 将颜色添加到Marker中
        marker.colors.push_back(color);
    }

    // 发布Marker消息
    marker_pub.publish(marker);
}

void gvf_manager::KinoPathCallback(const ros::TimerEvent& event)
{
    if (!use_kinopath_) return;  // 如果不使用动力学路径，则不处理
    auto& pm = swarmParticlesManager[0];
    if (!pm.receive_startpt) return;
    // 确定起点和初始速度
    Eigen::Vector3d start_pt;
    Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();  // 初始速度默认为0
    
    if (pm.is_first_kinogoal) {
        start_pt = Eigen::Vector3d(odom_.x(), odom_.y(), odom_.z());
        pm.is_first_kinogoal = false;
    } else {
        // 获取上一次的动力学轨迹
        std::vector<Eigen::Vector3d> last_path = pm.last_path;
        if (!last_path.empty()) {
            double min_dist = std::numeric_limits<double>::max();
            int nearest_idx = 0;
            Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
            
            // 找到当前位置在路径上的最近点
            for (size_t i = 0; i < last_path.size(); ++i) {
                double dist = (last_path[i] - current_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_idx = i;
                }
            }
            
            // 设置起点和初始速度
            start_pt = last_path[nearest_idx];
            
            // 计算路径方向（使用下一个点）
            if (nearest_idx < last_path.size() - 1) {
                Eigen::Vector3d direction = (last_path[nearest_idx + 1] - last_path[nearest_idx]).normalized();
                // 获取GVF增益
                double gvf_gain = pm.gvf_->gvf_.K1_;
                // 设置速度大小和方向
                start_vel = direction * gvf_gain;
            }
        } else {
            start_pt = Eigen::Vector3d(odom_.x(), odom_.y(), odom_.z());
        }
    }
    // 设置初始状态
    Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();  // 初始加速度
    Eigen::Vector3d original_goal = pm.goal_pt;
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();    // 目标速度

    // 调整目标点，确保在horizon_范围内
    Eigen::Vector3d end_pt = original_goal;
    double path_dist_to_goal = (original_goal - start_pt).norm();
    double horizon = pm.kino_path_finder_->horizon_;  // 获取horizon_参数

    // 添加终点衰减机制
    const double decay_start_dist = 1.0;  // 开始衰减的距离阈值
    const double stop_dist = 0.1;         // 停止的距离阈值
    
    if (path_dist_to_goal < decay_start_dist) {
        // 计算衰减系数
        double decay_factor = std::max(0.0, (path_dist_to_goal - stop_dist) / (decay_start_dist - stop_dist));
        
        // 对速度进行衰减
        start_vel *= decay_factor;
        
        // 如果非常接近终点，直接设置速度为零
        if (path_dist_to_goal < stop_dist) {
            start_vel = Eigen::Vector3d::Zero();
            end_vel = Eigen::Vector3d::Zero();
        }
    }

    if (path_dist_to_goal > horizon) {
        // 计算方向向量
        Eigen::Vector3d direction = (original_goal - start_pt).normalized();
        // 计算horizon范围内的点
        end_pt = start_pt + direction * horizon;
        
        // 检查该点是否被占据
        if (pm.sdf_map_->getInflateOccupancy(end_pt) == 1) {
            // 如果被占据，在horizon范围内寻找最近的可达点
            double search_radius = horizon;
            double angle_step = M_PI / 8;  // 22.5度
            bool found_valid_point = false;
            
            for (double angle = 0; angle < 2 * M_PI; angle += angle_step) {
                for (double r = search_radius; r > 0; r -= 0.1) {  // 从外向内搜索
                    Eigen::Vector3d test_point = start_pt + Eigen::Vector3d(
                        r * cos(angle),
                        r * sin(angle),
                        0
                    );
                    
                    if (pm.sdf_map_->getInflateOccupancy(test_point) == 0) {
                        end_pt = test_point;
                        found_valid_point = true;
                        break;
                    }
                }
                if (found_valid_point) break;
            }
        }
    }

    int result = pm.kino_path_finder_->search(start_pt, start_vel, start_acc, 
                                            end_pt, end_vel, false);
    
    // 检查搜索结果
    if (result == KinodynamicAstar::NO_PATH) {
        ROS_WARN("[GVF] No valid path found, skipping trajectory generation"); 
        return;
    }

    std::vector<Eigen::Vector3d> kino_path = pm.kino_path_finder_->getKinoTraj(0.01);  // 0.01s的时间间隔
    pm.last_path = kino_path;

    // 发布动力学路径
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : kino_path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = pt.z();
        path_msg.poses.push_back(pose);
    }
    kino_path_pub.publish(path_msg);
}

void gvf_manager::AstarExecCallback(const ros::TimerEvent& event) 
{   
    ros::Time t1 = ros::Time::now();
    
    if (use_kinopath_) return;  // 如果使用动力学路径，则不处理A*路径
    auto& pm = swarmParticlesManager[0];  
    if (!pm.receive_startpt) return;
          
    /*----------- ① A* 搜索路径 -----------*/
    Eigen::Vector3d start_pt;
    
    if (pm.is_first_goal) {
        // 第一次接收到目标点时，使用当前位置作为起点
        start_pt = Eigen::Vector3d(odom_.x()+0.000001, odom_.y()+0.000001, 1.0);
        pm.is_first_goal = false;
    } else {
        // 不是第一次时，找到上一次路径上与当前位置最近的点作为起点
        std::vector<Eigen::Vector3d> last_path = pm.geo_path_finder_->getPath();
        if (!last_path.empty()) {
            double min_dist = std::numeric_limits<double>::max();
            Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
            
            for (const auto& pt : last_path) {
                double dist = (pt - current_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    start_pt = pt;
                }
            }
        } else {
            // 如果没有上一次路径，使用当前位置
            start_pt = Eigen::Vector3d(odom_.x()+0.000001, odom_.y()+0.000001, 1.0);
        }
    }
    
    Eigen::Vector3d goal_pt = pm.goal_pt;  
         
    //do traj opt here
    Eigen::MatrixXd initial_state(3,3),terminal_state(3,3);//初始，结束P V A  
    Eigen::Vector3d end_pt, start_v, end_v, start_a;
    std::vector<Eigen::Vector3d> initial_ctrl_ps;

    ros::Time t2 = ros::Time::now();
    pm.geo_path_finder_->reset();
    pm.geo_path_finder_->search(start_pt, goal_pt, false, -1.0);
    std::vector<Eigen::Vector3d> path_points = pm.geo_path_finder_->getprunePath();   
    std::vector<Eigen::Vector3d> raw_path = pm.geo_path_finder_->getPath();   
    visualizePath(path_points, path_vis, pm.index); 
    ros::Time t3 = ros::Time::now();

    int num_points_to_take = std::min(static_cast<int>(path_points.size()), 10);
    for (int i = 0; i < num_points_to_take; ++i) {
        initial_ctrl_ps.push_back(path_points[i]);
    }
    if (initial_ctrl_ps.empty()) {
        ROS_INFO("[DEBUG] No new control points, publishing last trajectory");
        return;
    }
    end_pt = initial_ctrl_ps.back();    
    initial_state <<    start_pt(0), start_pt(1), start_pt(2),
                            0.0, 0.0,0.0,
                            0.0, 0.0,0.0;
    terminal_state <<   end_pt(0), end_pt(1),end_pt(2),
                            0.0, 0.0,0.0,
                            0.0, 0.0,0.0;
    pm.bspline_opt_->set3DPath2(initial_ctrl_ps);
    pm.spline_->setIniandTerandCpsnum(initial_state,terminal_state,pm.bspline_opt_->cps_num_);
    if(pm.bspline_opt_->cps_num_ <= 2*pm.spline_->p_)
    {
        // 如果控制点数量不足，发布path
        if (pm.last_traj.rows() > 0) {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "world";
            path_msg.header.stamp = ros::Time::now();

            for (auto pt:raw_path) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = pt.x();
                pose.pose.position.y = pt.y();
                pose.pose.position.z = pt.z();
                path_msg.poses.push_back(pose);
            }
            path_pub.publish(path_msg);
        }
        return;
    }
    
    UniformBspline spline = *pm.spline_;
    pm.bspline_opt_->setSplineParam(spline);
    pm.bspline_opt_->optimize();

    pm.spline_->setControlPoints(pm.bspline_opt_->control_points_);
    pm.spline_->getT();
    UniformBspline p = *pm.spline_;
    //traj
    Eigen::MatrixXd p_ = p.getTrajectory(p.time_);
    // 保存当前轨迹
    pm.last_traj = p_;
    ros::Time t4 = ros::Time::now();

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < p_.rows(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p_(i, 0);
        pose.pose.position.y = p_(i, 1);
        pose.pose.position.z = p_(i, 2);
        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
    ros::Time t5 = ros::Time::now();

    ROS_INFO("Timing: A* search: %.3f ms, Traj opt: %.3f ms, Publish: %.3f ms",
             (t3-t2).toSec()*1000, (t4-t3).toSec()*1000, (t5-t4).toSec()*1000);
}

std::vector<Eigen::Vector3d>
gvf_manager::correctPathToCenter(const std::vector<Eigen::Vector3d>& raw_path)
{
std::vector<Eigen::Vector3d> center;
const double step = 0.05;      

for (auto p : raw_path)
{
    for (int i = 0; i < 7; ++i)                      // 迭代 7 次 ≈ 3 格
    {
    Eigen::Vector3d g = esdfGrad(p);               // ∇d(p)
    if (g.squaredNorm() < 1e-6) break;             // 到脊线
    p += step * g.normalized();                    // 上山
    }
    center.push_back(p);
}
return center;     // Γ_c
}

}