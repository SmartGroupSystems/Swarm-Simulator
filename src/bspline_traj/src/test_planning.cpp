#include <bspline_race/bspline_race.h>

using namespace FLAG_Race;

ros::Publisher traj_vis;
ros::Publisher traj_puber;
ros::Publisher waypoint_vis;
    
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "bspline_traj");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);

    ros::Publisher waypoint_vis = nh.advertise<visualization_msgs::Marker>("/waypoint_vis", 10, true);
    ros::Publisher traj_puber = nh.advertise<bspline_race::BsplineTraj>("/bspline_traj", 10, true);
    ros::Publisher traj_vis = nh.advertise<visualization_msgs::Marker>("/traj_vis", 10, true);

    // 获取文件路径
    std::string traj_file_path;
    if (!nh.getParam("traj_file", traj_file_path)) {
        ROS_ERROR("Failed to get param 'traj_file'");
        return -1;
    }

    // 读取文件并存储点
    std::ifstream file(traj_file_path);
    std::vector<Point> waypoints;
    if (file.is_open()) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            char ch; // 用于读取方括号和逗号
            Point p;
            ss >> ch >> p.x >> ch >> p.y >> ch >> p.z >> ch;
            waypoints.push_back(p);
        }
        file.close();
    } else {
        ROS_ERROR("Unable to open file: %s", traj_file_path.c_str());
        return -1;
    }

    // 设置Marker消息
    visualization_msgs::Marker points;
    points.header.frame_id = "world"; // 或者你想要的任何坐标系
    points.header.stamp = ros::Time::now();
    points.ns = "waypoints";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.scale.x = 0.2; // 点的大小
    points.scale.y = 0.2;
    points.scale.z = 0.2;

    // 设置点的颜色
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    // 填充点
    for (const auto& waypoint : waypoints) {
        geometry_msgs::Point p;
        p.x = waypoint.x;
        p.y = waypoint.y;
        p.z = waypoint.z;
        points.points.push_back(p);
    }

    //计算轨迹
    traj_ = manager.getSmoothTraj(waypoints);

    // 设置轨迹可视化 Marker 消息
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "world"; // 使用适当的坐标系
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.id = 1;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.scale.x = 0.1; // 线条的宽度

    // 设置线条的颜色
    traj_marker.color.r = 1.0f;
    traj_marker.color.g = 0.0f;
    traj_marker.color.b = 0.0f;
    traj_marker.color.a = 1.0;

    // 固定的高度值
    double fixed_z = 0.8; // 你可以设置为任何合适的值

    // 填充轨迹点
    for (const auto& pose_stamped : traj_.position) {
        geometry_msgs::Point p;
        p.x = pose_stamped.pose.position.x;
        p.y = pose_stamped.pose.position.y;
        p.z = fixed_z; // 使用固定的高度值
        traj_marker.points.push_back(p);
    }

    // 发布
    waypoint_vis.publish(points);
    traj_puber.publish(traj_);
    traj_vis.publish(traj_marker);

    ros::spin();
    return 0;
}
