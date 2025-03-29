#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_goal_publisher");
  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // 设置随机数生成器
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> dist_x(0.0, 30.0);
  std::uniform_real_distribution<double> dist_y(0.0, 30.0);
  std::uniform_real_distribution<double> dist_z(1.0, 7.0);

  ros::Rate rate(10);  // 每0.1秒发布一个点

  for (int i = 0; i < 60 && ros::ok(); ++i) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "world";  // 你可以根据需要修改坐标系

    goal_msg.pose.position.x = dist_x(generator);
    goal_msg.pose.position.y = dist_y(generator);
    goal_msg.pose.position.z = dist_z(generator);

    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;

    goal_pub.publish(goal_msg);
    ROS_INFO_STREAM("Published random goal #" << i + 1);

    rate.sleep();
  }

  return 0;
}
