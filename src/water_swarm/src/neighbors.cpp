#include "neighbors.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& uav_name) {
    // 处理odom消息
    ROS_INFO("Received odom from %s", uav_name.c_str());
    // 你可以在这里添加更多的逻辑
    
}

void subscribeOdom(const std::string &topic_name, ros::NodeHandle &nh) {
    subscribers[topic_name] = nh.subscribe<nav_msgs::Odometry>(
        topic_name, 
        1000, 
        boost::bind(odomCallback, _1, topic_name)
    );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neighbors_node");
    ros::NodeHandle nh("~");

    nh.param("neighbor_dist",neighbor_dist_,1.0);

    ROS_INFO("Neighbors node has started.");

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (const auto &info : master_topics) {
        if (info.datatype == "nav_msgs/Odometry") {
            // Check if topic name matches the pattern /uav(i)/sim/odom
            std::regex topic_pattern("/uav\\d+/sim/odom");
            if (std::regex_match(info.name, topic_pattern)) {
                subscribeOdom(info.name, nh);
                ROS_INFO("Subscribed to %s", info.name.c_str());
            }
        }
    }

    ros::spin();
    return 0;
}

