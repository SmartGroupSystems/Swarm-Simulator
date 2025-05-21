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
        initCallback(nh);
        InitGvf(nh);
    }
    gvf_manager::~gvf_manager() {}

    void gvf_manager::initCallback(ros::NodeHandle &nh)
    {
        exec_timer = nh.createTimer(ros::Duration(0.10), &gvf_manager::AstarExecCallback, this);
        force_pub  = nh.advertise<common_msgs::Swarm_particles>("/gvf_force", 10);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &gvf_manager::goalCallback, this);
        path_vis = nh.advertise<visualization_msgs::Marker>("/path_vis", 10);
        
    } 

void gvf_manager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("[GVF] receive goal (%.2f, %.2f, %.2f)",
             msg->pose.position.x,
             msg->pose.position.y,
             msg->pose.position.z);

    Eigen::Vector3d goal_pt(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );

    for (auto& manager : swarmParticlesManager) {
        manager.goal_pt = goal_pt;
    }
}

void gvf_manager::AstarExecCallback(const ros::TimerEvent& event) 
{   
    auto& pm = swarmParticlesManager[0];        
    /*----------- ① A* 搜索粗路径 -----------*/
    Eigen::Vector3d start_pt( 0.00001, 0.00001, 1.0 );
    Eigen::Vector3d goal_pt = pm.goal_pt;        

    pm.geo_path_finder_->reset();
    pm.geo_path_finder_->search(start_pt, goal_pt, false, -1.0);
    std::vector<Eigen::Vector3d> raw_path = pm.geo_path_finder_->getPath();         
    std::vector<Eigen::Vector3d> center_pts = correctPathToCenter(raw_path);
    visualizePath(center_pts, path_vis, pm.index); 
    
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : center_pts) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = pt.z();
        path_msg.poses.push_back(pose);
    }
    pm.path_pub.publish(path_msg);

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
            auto geo_path_finder_ = std::make_shared<Astar>();
            geo_path_finder_->setParam(nh);
            geo_path_finder_->setEnvironment(edt_environment_);
            geo_path_finder_->init();

            // dynamic a*
            auto kino_path_finder_ = std::make_shared<KinodynamicAstar>();
            
            // gvf
            auto gvf_ = std::make_shared<gvf>();
            gvf_->init(nh, particle_base, odom_topic_, cloud_topic_);

            ros::Publisher path_pub = nh.advertise<nav_msgs::Path>( particle_base + "/path", 1);

            gvfManager pm {
                particle_base,
                sdf_map_,
                edt_environment_,
                geo_path_finder_,
                kino_path_finder_,
                gvf_,
                ros::Time::now(),     // curr_time
                ros::Time(0),         // last_time
                true, 
                path_pub
            };

            swarmParticlesManager.push_back(pm);  // 将实例存入向量

            std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught while initializing environments for %s", e.what());
        }  
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

    std::vector<Eigen::Vector3d>
    gvf_manager::correctPathToCenter(const std::vector<Eigen::Vector3d>& raw_path)
    {
    std::vector<Eigen::Vector3d> center;
    const double step = 0.05;      

    for (auto p : raw_path)
    {
        for (int i = 0; i < 6; ++i)                      // 迭代 6 次 ≈ 3 格
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