#include "neighbors.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name) {
    // ROS_INFO("Received odom from %s", uav_name.c_str());
    water_swarm::Odom current_odom;
    current_odom.position.x = msg->pose.pose.position.x;
    current_odom.position.y = msg->pose.pose.position.y;
    current_odom.position.z = msg->pose.pose.position.z;
    current_odom.velocity.x = msg->twist.twist.linear.x;
    current_odom.velocity.y = msg->twist.twist.linear.y;
    current_odom.velocity.z = msg->twist.twist.linear.z;
    
    std_msgs::String uav_name;
    uav_name.data = extractUavName(name);

    // 检查broadcast_odom是否为空
    if (broadcast_odom.empty()) {
        broadcast_odom.push_back(current_odom);
        uav_names.push_back(uav_name);
    } else {
        // 检查距离并更新broadcast_odom,uav_names
        for (auto it = broadcast_odom.begin(); it != broadcast_odom.end(); ) {
            double dist = sqrt(pow(it->position.x - current_odom.position.x, 2) +
                               pow(it->position.y - current_odom.position.y, 2) +
                               pow(it->position.z - current_odom.position.z, 2));
            if (dist < threshold_dist_) {
                // 计算需要擦除的元素在向量中的索引
                size_t index = std::distance(broadcast_odom.begin(), it);
                it = broadcast_odom.erase(it);
                uav_names.erase(uav_names.begin() + index);
                break;
            } else {
                ++it;
            }
        }
        broadcast_odom.push_back(current_odom);
        uav_names.push_back(uav_name);
    }
    water_swarm::OdomBroadcast odom_broadcast_msg;
    odom_broadcast_msg.header.frame_id  = "world";
    odom_broadcast_msg.header.stamp     = ros::Time::now();
    odom_broadcast_msg.drone_names      = uav_names; 
    odom_broadcast_msg.OdomBroadcast    = broadcast_odom;
    // ROS_INFO("Size of broadcast_odom: %lu", broadcast_odom.size());

    //find neighbors
    std::vector<water_swarm::Odom> neighbors;
    for (const auto& neighbor_odom : broadcast_odom) {
        double dist = sqrt(pow(neighbor_odom.position.x - current_odom.position.x, 2) +
                           pow(neighbor_odom.position.y - current_odom.position.y, 2) +
                           pow(neighbor_odom.position.z - current_odom.position.z, 2));
        if (dist <= neighbor_dist_ && dist != 0) {  // dist != 0 确保不将当前odom作为其自己的邻居
            neighbors.push_back(neighbor_odom);
        }
    }

    // 创建OdomWithNeighbors消息
    water_swarm::OdomWithNeighbors odom_with_neighbors_msg;
    odom_with_neighbors_msg.header.stamp    = ros::Time::now();
    odom_with_neighbors_msg.header.frame_id = "world"; // 或者其他适合的frame_id
    odom_with_neighbors_msg.drone_name      = uav_name;
    odom_with_neighbors_msg.myOdom = current_odom;
    odom_with_neighbors_msg.neighborsOdom = neighbors;

    //pub odom_with_neighbors  odom_broadcast
    odom_neighbors_publishers[name].publish(odom_with_neighbors_msg);
    odomBroadcast_pub.publish(odom_broadcast_msg);
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

    nh.param("neighbor_dist",   neighbor_dist_, 10.05);
    nh.param("threshold_dist",  threshold_dist_,0.1);

    ROS_INFO("Neighbors node has started.");

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (const auto &info : master_topics) {
        if (info.datatype == "nav_msgs/Odometry") {
            // Check if topic name matches the pattern /uav(i)/sim/odom
            std::regex topic_pattern("/uav\\d+/sim/odom");
            if (std::regex_match(info.name, topic_pattern)) {
                subscribeOdom(info.name, nh);
                ROS_INFO("Subscribed to %s, uav name:%s", info.name.c_str(),extractUavName(info.name).c_str());
                // 为每个UAV创建一个独立的Publisher
                std::string publisher_topic = extractUavName(info.name) + "_odomWithNeighbors";  // 创建基于UAV名称的话题
                odom_neighbors_publishers[info.name] = nh.advertise<water_swarm::OdomWithNeighbors>(publisher_topic, 1000);
                ROS_INFO("Publisher created for topic: %s", publisher_topic.c_str());
            }
        }
    }

    odomBroadcast_pub = nh.advertise<water_swarm::OdomBroadcast>("/odomBroadcast",1000);


    ros::spin();
    return 0;
}

