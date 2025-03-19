#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <vector>
#include <random>

// 机器人数量
const int NUM_ROBOTS = 169;
// 形成五角星的顶点
const int STAR_POINTS = 5;
// 置信度收敛阈值
const double CONFIDENCE_THRESHOLD = 0.8;
// 传播步长
const double ALPHA = 0.05;
// 交互影响因子
const double BETA = 0.02;
// 迭代更新间隔
const double UPDATE_RATE = 0.1; 

// 机器人状态
struct Robot {
    Eigen::Vector2d position;   // 当前位置
    Eigen::Vector2d target;     // 目标位置（初始化为空）
    double confidence;          // 置信度
};

// 机器人群体
std::vector<Robot> robots;

std::vector<Eigen::Vector2d> generate_uniform_square(double side_length = 5.0, double spacing = 0.5) {
    std::vector<Eigen::Vector2d> square_points;
    
    int num_per_side = static_cast<int>(side_length / spacing) + 1; // 计算每行/列的点数
    double start = -side_length / 2.0;  // 让正方形中心在 (0,0)

    for (int i = 0; i < num_per_side; ++i) {
        for (int j = 0; j < num_per_side; ++j) {
            double x = start + i * spacing;
            double y = start + j * spacing;
            square_points.emplace_back(x, y);
        }
    }

    return square_points;
}


// 初始化机器人（部分机器人知道目标形状）
void initialize_robots() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dis(0, 2 * M_PI);  // 角度 0 ~ 2π
    std::uniform_real_distribution<> radius_dis(0, 10);       // 半径 0 ~ 10

    // 生成均匀的正方形目标队形
    std::vector<Eigen::Vector2d> square_shape = generate_uniform_square(6.0, 0.5);

    // 选取最多 NUM_ROBOTS 个目标点
    int num_targets = std::min(NUM_ROBOTS, static_cast<int>(square_shape.size()));

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        Robot r;
        double angle = angle_dis(gen);  
        double radius = radius_dis(gen);  

        // 初始随机分布在圆形区域
        r.position = Eigen::Vector2d(radius * cos(angle), radius * sin(angle));

        // 置信度设定
        r.confidence = (i < num_targets) ? 1.0 : 0.0;

        // 机器人均匀分布在正方形
        if (i < num_targets) {
            r.target = square_shape[i];
        } else {
            r.target = Eigen::Vector2d::Zero();  // 其余机器人目标未知
        }

        robots.push_back(r);
    }
}


// 更新机器人目标（信息传播）
void update_robot_targets() {
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        if (robots[i].confidence > CONFIDENCE_THRESHOLD) continue; // 已稳定

        Eigen::Vector2d new_target = Eigen::Vector2d::Zero();
        double sum_confidence = 0.0;

        for (int j = 0; j < NUM_ROBOTS; ++j) {
            if (i == j) continue;
            double distance = (robots[i].position - robots[j].position).norm();
            if (distance < 5.0 && robots[j].confidence > 0.0) { // 只考虑邻近且已有目标的机器人
                new_target += robots[j].confidence * robots[j].target;
                sum_confidence += robots[j].confidence;
            }
        }

        if (sum_confidence > 0.0) {
            robots[i].target = new_target / sum_confidence;
            robots[i].confidence = sum_confidence / 10.0; // 置信度累积
        }
    }
}


// 位置调整
void update_robot_positions() {
    for (auto &robot : robots) {
        if (robot.confidence < CONFIDENCE_THRESHOLD) continue;

        // 朝目标移动
        Eigen::Vector2d error = robot.target - robot.position;
        robot.position += ALPHA * error;

        // 受邻居影响（扩散模型）
        Eigen::Vector2d neighbor_adjustment = Eigen::Vector2d::Zero();
        int count = 0;
        for (const auto &neighbor : robots) {
            if ((neighbor.position - robot.position).norm() < 5.0) {
                neighbor_adjustment += neighbor.position - robot.position;
                count++;
            }
        }
        if (count > 0) {
            robot.position += BETA * (neighbor_adjustment / count);
        }
    }
}

// 发布可视化信息
void publish_markers(ros::Publisher &marker_pub) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robots";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    for (const auto &robot : robots) {
        geometry_msgs::Point p;
        p.x = robot.position.x();
        p.y = robot.position.y();
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker_pub.publish(marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "star_formation");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate loop_rate(10);

    initialize_robots();

    while (ros::ok()) {
        update_robot_targets();
        update_robot_positions();
        publish_markers(marker_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
