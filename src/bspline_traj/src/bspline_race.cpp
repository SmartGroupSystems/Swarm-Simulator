#include "bspline_race/bspline_race.h"

namespace FLAG_Race

{
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        // testInit(nh);
        nh.param("fsm/planInterval", planInterval, -1.0);
        initCallback(nh);
        parallelInit(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::initCallback(ros::NodeHandle &nh)
    {
        particles_sub = nh.subscribe("/swarm_particles", 1000, &plan_manager::particlesCallback,this);
        traj_timer = nh.createTimer(ros::Duration(0.01), &plan_manager::timerCallback,this);
        traj_puber = nh.advertise<common_msgs::Swarm_traj>("/swarm_traj", 10, true);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &plan_manager::goalCallback, this);
        path_vis = nh.advertise<visualization_msgs::Marker>("/path_vis", 10);
        traj_vis = nh.advertise<visualization_msgs::Marker>("/traj_vis", 10);
        lastPlanTime = ros::Time::now();
        lastWaitOutputTime = ros::Time::now(); 
    }

    void plan_manager::particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
    {
        if (isFirstCall) {
            init_particles = *msg; 
            isFirstCall = false; 
        } 
        else {
            current_particles = *msg; 
        }
    }

    void plan_manager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("Received goal: [%f, %f, %f]", msg->pose.position.x, 
                                                msg->pose.position.y, 
                                                msg->pose.position.z);
        particles_goal = init_particles;                                            
        for (size_t i = 0; i < particles_goal.particles.size(); i++)
        {
            particles_goal.particles[i].position.x += msg->pose.position.x;
            particles_goal.particles[i].position.y += msg->pose.position.y;
            particles_goal.particles[i].position.z += msg->pose.position.z;
            ROS_INFO("Particle %d's goal: %f, %f,  %f",particles_goal.particles[i].index,
                                                       particles_goal.particles[i].position.x,
                                                       particles_goal.particles[i].position.y,
                                                       particles_goal.particles[i].position.z);
        }
        
        receive_goal = true;
    }

    void plan_manager::timerCallback(const ros::TimerEvent&) 
    {
        ros::Time currentTime = ros::Time::now(); 

        if (receive_goal)
        {   
            receive_goal = false;
            //this func...
            ROS_INFO("\033[1;32m START OPTIMIZE_TRAJ! \033[0m");
            optTraj();
            ros::Time optfinishTime = ros::Time::now(); 
            double optcostTime = (optfinishTime - currentTime).toSec();
            ROS_INFO("\033[1;32m OPTIMIZE_FINISH, TOTAL TIME COST: %Fs \033[0m",optcostTime);
            exec_traj = true;
        }
        else if (exec_traj)
        {
            double waitElapsedTime = (currentTime - lastWaitOutputTime).toSec();
            if (waitElapsedTime >= 1.0)
            {
                ROS_INFO("EXEC_TRAJ");
                lastWaitOutputTime = currentTime;
            }
        }
        else
        {
            double waitElapsedTime = (currentTime - lastWaitOutputTime).toSec();
            if (waitElapsedTime >= 1.0)
            {
                ROS_INFO("WAIT_TARGET");
                lastWaitOutputTime = currentTime;
            }
        }
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
    
   
    void plan_manager::processParticle(size_t index, const common_msgs::Swarm_particles& particles, 
                                       const common_msgs::Swarm_particles& particles_goal, 
                        std::vector<particleManager>& swarmParticlesManager, 
                        ros::Publisher& path_vis, std::mutex& mtx) {
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        Eigen::Vector3d start_pt, end_pt, start_v, start_a;

        // Assign start_pt using current_particles' position
        start_pt.x() = particles.particles[index].position.x+0.000001;//if start is zero, a_star bug
        start_pt.y() = particles.particles[index].position.y+0.000001;//
        start_pt.z() = particles.particles[index].position.z;
        start_v.x()  = particles.particles[index].velocity.x;
        start_v.y()  = particles.particles[index].velocity.y;
        start_v.z()  = particles.particles[index].velocity.z;
        start_a.x()  = particles.particles[index].acceleration.x;
        start_a.y()  = particles.particles[index].acceleration.y;
        start_a.z()  = particles.particles[index].acceleration.z;

        // Find the matching particle in particles_goal based on the index
        int current_index = particles.particles[index].index;
        for (const auto& goal_particle : particles_goal.particles) {
            if (goal_particle.index == current_index) {
                // Assign end_pt using goal_particle's position
                end_pt.x() = goal_particle.position.x;
                end_pt.y() = goal_particle.position.y;
                end_pt.z() = goal_particle.position.z;
                break;
            }
        }

        //  cout<< start_pt.transpose()<< "     "<< end_pt.transpose()<<endl;

        initial_state <<    start_pt(0), start_pt(1),
                            start_v(0),  start_v(1),
                            start_a(0),  start_a(1);

        terminal_state <<   end_pt(0), end_pt(1),
                            0.0, 0.0,
                            0.0, 0.0;

        // Reset pathfinder and search for the path
        swarmParticlesManager[index].geo_path_finder_->reset();
        swarmParticlesManager[index].geo_path_finder_->search(start_pt, end_pt, false, -1.0);
        std::vector<Eigen::Vector3d> path_points = swarmParticlesManager[index].geo_path_finder_->getprunePath();
            visualizePath(path_points, path_vis, swarmParticlesManager[index].particle_index);
        if (path_points.size()==0)
        {
           return;
        }
        swarmParticlesManager[index].bspline_opt_->set3DPath(path_points);

        swarmParticlesManager[index].spline_->setIniandTerandCpsnum(initial_state,terminal_state,
                                                            swarmParticlesManager[index].bspline_opt_->cps_num_);
        if(swarmParticlesManager[index].bspline_opt_->cps_num_ == 2*swarmParticlesManager[index].spline_->p_)
        {
            return;
        }
        UniformBspline spline = *swarmParticlesManager[index].spline_;
        swarmParticlesManager[index].bspline_opt_->setSplineParam(spline);
        // swarmParticlesManager[index].bspline_opt_->optimize();


   
    }

    void plan_manager::optTraj()
    {
        size_t num_particles = current_particles.particles.size();
        std::vector<std::thread> threads;
        std::mutex mtx;

        // Create a thread for each particle
        for (size_t i = 0; i < num_particles; ++i) {
            threads.emplace_back(std::bind(&plan_manager::processParticle, this, i, 
                                            std::cref(current_particles), std::cref(particles_goal),
                                            std::ref(swarmParticlesManager), std::ref(path_vis), std::ref(mtx)));
        }

        // Join all threads
        for (auto& thread : threads) {
            thread.join();
        }
    }

    // void plan_manager::optTraj()
    // {
    //     for (size_t i = 0; i < init_particles.particles.size(); i++)
    //     {
    //         Eigen::Vector3d start_pt, end_pt;
    //         // Assign start_pt using current_particles' position
    //         start_pt.x() = init_particles.particles[i].position.x;
    //         start_pt.y() = init_particles.particles[i].position.y;
    //         start_pt.z() = init_particles.particles[i].position.z;

    //         // Find the matching particle in particles_goal based on the index
    //         int current_index = init_particles.particles[i].index;
    //         for (const auto& goal_particle : particles_goal.particles) {
    //         if (goal_particle.index == current_index) {
    //             // Assign end_pt using goal_particle's position
    //                 end_pt.x() = goal_particle.position.x;
    //                 end_pt.y() = goal_particle.position.y;
    //                 end_pt.z() = goal_particle.position.z;
    //                 break;
    //             }
    //         }

    //         swarmParticlesManager[i].geo_path_finder_->reset();
    //         swarmParticlesManager[i].geo_path_finder_->search(start_pt,end_pt,false,-1.0);
    //         std::vector<Eigen::Vector3d> path_points = swarmParticlesManager[i].geo_path_finder_->getprunePath();
    //         visualizePath(path_points, path_vis, swarmParticlesManager[i].particle_index);


    //     }
    // }

    void plan_manager::parallelInit(ros::NodeHandle &nh) {
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

                    std::cout << "\033[1;33m" << particle_base << "  init: "<< "\033[0m" << std::endl;

                    //EDT & MAP
                    auto sdf_map_ = std::make_shared<SDFMap>();
                    sdf_map_->initMap(nh, particle_base, odom_topic, cloud_topic);
                    auto edt_environment_ = std::make_shared<EDTEnvironment>();
                    edt_environment_->setMap(sdf_map_);
                    
                    //ASTAR
                    auto geo_path_finder_ = std::make_shared<Astar>();
                    geo_path_finder_->setParam(nh);
                    geo_path_finder_->setEnvironment(edt_environment_);
                    geo_path_finder_->init();

                    //OPT
                    auto bspline_opt_ = std::make_shared<bspline_optimizer>();
                    bspline_opt_->init(nh);
                    bspline_opt_->setEnvironment(edt_environment_);

                    //UNIFORM BSPLINE
                    auto spline_ = std::make_shared<UniformBspline>();
                    spline_->init(nh);

                    // sdf_maps.push_back(sdf_map_);
                    // edt_environments.push_back(edt_environment_);
                    // swarm_astar.push_back(geo_path_finder_);
                    // swarm_opt.push_back(bspline_opt_);
                    // swarm_bspline.push_back(spline_);
                    particleManager pm {
                        particle_index,
                        sdf_map_,
                        edt_environment_,
                        geo_path_finder_,
                        bspline_opt_,
                        spline_
                    };

                    swarmParticlesManager.push_back(pm);  // 将实例存入向量

                    std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

                } catch (const std::exception& e) {
                    ROS_ERROR("Exception caught while initializing environments for %s: %s", info.name.c_str(), e.what());
                } catch (...) {
                    ROS_ERROR("Unknown exception caught during initialization for %s", info.name.c_str());
                }
            }
        }
    }

    void plan_manager::visualizePath(const std::vector<Eigen::Vector3d>& path_points, ros::Publisher& marker_pub, const std::string& particle_index) {
        // Lock the mutex to safely visualize the path
        std::lock_guard<std::mutex> lock(mtx);  // 锁定
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


    void plan_manager::testInit(ros::NodeHandle& nh)
    {
        ros::NodeHandle node_;
        node_ = nh;
        ROS_INFO("Test Init~~~~~!");
    }

}