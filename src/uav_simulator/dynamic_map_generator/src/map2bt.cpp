#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    octomap::OcTree tree(0.1);  // 分辨率为 0.1m，可调

    for (auto& pt : pcl_cloud.points)
    {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            tree.updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
    }

    tree.updateInnerOccupancy();

    // 保存为 .bt 文件
    tree.writeBinary("mock_map.bt");
    ROS_INFO("OctoMap written to mock_map.bt");

    ros::shutdown();  // 只运行一次
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_octomap");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/mock_map", 1, cloudCallback);

    ros::spin();
    return 0;
}
