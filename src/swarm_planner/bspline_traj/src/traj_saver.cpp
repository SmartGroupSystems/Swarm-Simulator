#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <map>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

double flight_height;
// 存储初始位置
std::map<int, std::vector<double>> initial_positions;
// 存储文件流
std::map<int, std::ofstream> file_streams;

// 获取当前系统时间的字符串表示（年月日_小时分钟）
std::string getCurrentFolderName() {
    auto now = std::time(nullptr);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now), "%Y_%m_%d_%H_%M");
    return oss.str();
}

// 回调函数处理odom信息
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int particle_id) {
    // 获取初始位置
    if (initial_positions.find(particle_id) == initial_positions.end()) {
        initial_positions[particle_id] = {msg->pose.pose.position.x, 
                                          msg->pose.pose.position.y, 
                                          msg->pose.pose.position.z};
    }
    auto &init_pos = initial_positions[particle_id];

    // 计算相对位置
    double rel_x = msg->pose.pose.position.x - init_pos[0];
    double rel_y = msg->pose.pose.position.y - init_pos[1];
    double rel_z = msg->pose.pose.position.z; // z不需要改变

    // 获取速度
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    // 写入文件
    if (file_streams.find(particle_id) != file_streams.end()) {
        file_streams[particle_id] << std::fixed << std::setprecision(6)
                                  << rel_x << "," << rel_y << "," << rel_z << ","
                                  << vx << "," << vy << "," << vz << "\n";
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_saver");
    ros::NodeHandle nh("~");

    int particle_num;
    double wait_time;
    nh.param("wait_time", wait_time, 1.0);
    nh.param("particle_num", particle_num, 1); 
    nh.param("flight_height", flight_height, 0.5); 
    ROS_INFO("Waiting for %.2f seconds before starting...", wait_time);
    ros::Duration(wait_time).sleep();

    // 生成带时间戳的文件夹路径
    std::string base_folder = "/home/uav/water_swarm/src/swarm_planner/bspline_traj/traj/";
    std::string folder_name = getCurrentFolderName();
    std::string full_folder_path = base_folder + folder_name + "/";

    // 创建目录
    struct stat st;
    if (stat(full_folder_path.c_str(), &st) != 0) {
        if (mkdir(full_folder_path.c_str(), 0777) == -1) {
            ROS_ERROR("Failed to create folder: %s, errno: %d (%s)", 
                    full_folder_path.c_str(), errno, strerror(errno));
            return -1;
        }
    } else {
        // 目录已经存在
        if (!S_ISDIR(st.st_mode)) {
            ROS_ERROR("Path exists but is not a directory: %s", full_folder_path.c_str());
            return -1;
        }
    }
    ROS_INFO("Saving trajectories to folder: %s", full_folder_path.c_str());

    // 订阅话题
    std::vector<ros::Subscriber> subscribers;
    for (int i = 0; i < particle_num; ++i) { 
        std::string topic_name = "/particle" + std::to_string(i) + "/odom";
        std::string file_name = full_folder_path + std::to_string(i) + ".txt";

        // 打开文件流
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_name.c_str());
            continue;
        }
        file_streams[i] = std::move(ofs);
        ROS_INFO("Saving trajectory to file: %s", file_name.c_str());

        // 订阅话题
        subscribers.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name, 1000, boost::bind(odomCallback, _1, i)));
    }

    // 运行ROS主循环
    ros::spin();

    // 关闭所有文件流
    for (auto &entry : file_streams) {
        entry.second.close();
    }

    // 输出绿色大号字体提示
    std::cout << "\033[1;32mtraj saved.\033[0m" << std::endl;

    return 0;
}
