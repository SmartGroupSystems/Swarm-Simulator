#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <map>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

double flight_height;
std::map<int, std::vector<double>> initial_positions;
std::map<int, std::ofstream> file_streams;
int particle_num;

// 获取当前时间作为文件夹名
std::string getCurrentFolderName() {
    auto now = std::time(nullptr);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now), "%Y_%m_%d_%H_%M");
    return oss.str();
}

// ------------------ 新增：初始化初始位置 ------------------
void initMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    for (const auto& marker : msg->markers) {
        int id = marker.id;
        if (id >= 0 && id < particle_num) {
            initial_positions[id] = {
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            };
            ROS_INFO("Initialized position for particle %d: [%.2f, %.2f, %.2f]", 
                     id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        }
    }
}

// ------------------ 修改后的 odom 回调 ------------------
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int particle_id) {
    if (initial_positions.find(particle_id) == initial_positions.end()) {
        ROS_WARN_THROTTLE(5.0, "Initial position for particle %d not received yet.", particle_id);
        return;
    }

    const auto& init_pos = initial_positions[particle_id];

    double rel_x = msg->pose.pose.position.x - init_pos[0];
    double rel_y = msg->pose.pose.position.y - init_pos[1];
    double rel_z = msg->pose.pose.position.z;

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    if (file_streams.find(particle_id) != file_streams.end()) {
        file_streams[particle_id] << std::fixed << std::setprecision(6)
                                  << rel_x << "," << rel_y << "," << rel_z << ","
                                  << vx << "," << vy << "," << vz << "\n";
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_saver");
    ros::NodeHandle nh("~");

    double wait_time;
    nh.param("wait_time", wait_time, 1.0);
    nh.param("particle_num", particle_num, 1); 
    nh.param("flight_height", flight_height, 0.5); 

    ROS_INFO("Waiting for %.2f seconds before starting...", wait_time);
    ros::Duration(wait_time).sleep();

    std::string base_folder = "/home/uav/water_swarm/src/swarm_planner/bspline_traj/traj/";
    std::string folder_name = getCurrentFolderName();
    std::string full_folder_path = base_folder + folder_name + "/";

    struct stat st;
    if (stat(full_folder_path.c_str(), &st) != 0) {
        if (mkdir(full_folder_path.c_str(), 0777) == -1) {
            ROS_ERROR("Failed to create folder: %s, errno: %d (%s)", 
                    full_folder_path.c_str(), errno, strerror(errno));
            return -1;
        }
    } else if (!S_ISDIR(st.st_mode)) {
        ROS_ERROR("Path exists but is not a directory: %s", full_folder_path.c_str());
        return -1;
    }

    ROS_INFO("Saving trajectories to folder: %s", full_folder_path.c_str());

    std::vector<ros::Subscriber> subscribers;

    // 订阅每个粒子的 odom 话题
    for (int i = 0; i < particle_num; ++i) { 
        std::string topic_name = "/particle" + std::to_string(i) + "/odom";
        std::string file_name = full_folder_path + std::to_string(i) + ".txt";

        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_name.c_str());
            continue;
        }
        file_streams[i] = std::move(ofs);
        ROS_INFO("Saving trajectory to file: %s", file_name.c_str());

        subscribers.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name, 1000, boost::bind(odomCallback, _1, i)));
    }

    // ------------------ 新增：订阅初始位姿 ------------------
    ros::Subscriber marker_sub = nh.subscribe("/initial_pose", 1, initMarkerCallback);

    ros::spin();

    for (auto &entry : file_streams) {
        entry.second.close();
    }

    std::cout << "\033[1;32mtraj saved.\033[0m" << std::endl;

    return 0;
}
