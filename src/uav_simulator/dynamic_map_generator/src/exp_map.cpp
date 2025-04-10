#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

ros::Publisher map_pub;
int map_id;

PointCloud::Ptr generateBoundary(double length = 14.0, double width = 8.0, double height = 3.0) {
    PointCloud::Ptr cloud(new PointCloud);
    double resolution = 0.1;

    // Floor
    for (double x = 0; x <= length; x += resolution) {
        for (double y = 0; y <= width; y += resolution) {
            cloud->points.emplace_back(x, y, 0.0);
        }
    }

    // Left and Right walls (y = 0 and y = width)
    for (double z = 0; z <= height; z += resolution) {
        for (double x = 0; x <= length; x += resolution) {
            cloud->points.emplace_back(x, 0.0, z);
            cloud->points.emplace_back(x, width, z);
        }
    }

    return cloud;
}

PointCloud::Ptr generateMap1() {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    std::vector<Eigen::Vector2d> centers = {
        {4.0, 2.0}, {5.5, 5.5}, {7.0, 2.5}, {8.5, 6.0},
        {10.0, 3.0}, {11.5, 5.0}, {12.5, 2.0}
    };
    double size = 0.3;
    double height = 1.5;

    for (const auto& c : centers) {
        for (double x = c.x() - size / 2; x <= c.x() + size / 2; x += 0.05) {
            for (double y = c.y() - size / 2; y <= c.y() + size / 2; y += 0.05) {
                for (double z = 0; z <= height; z += 0.2) {
                    cloud->points.emplace_back(x, y, z);
                }
            }
        }
    }

    *cloud += *generateBoundary();
    return cloud;
}

PointCloud::Ptr generateMap2() {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    std::vector<Eigen::Vector2d> centers = {
        {4.5, 2.5}, {6.0, 6.0}, {7.5, 3.0}, {9.0, 5.5},
        {10.5, 2.5}, {12.0, 4.0}, {13.0, 6.5}
    };
    double size = 0.3;
    double height = 2.0;

    for (const auto& c : centers) {
        for (double x = c.x() - size / 2; x <= c.x() + size / 2; x += 0.05) {
            for (double y = c.y() - size / 2; y <= c.y() + size / 2; y += 0.05) {
                for (double z = 0; z <= height; z += 0.2) {
                    cloud->points.emplace_back(x, y, z);
                }
            }
        }
    }

    *cloud += *generateBoundary();
    return cloud;
}

PointCloud::Ptr generateMap3() {
    PointCloud::Ptr cloud = generateMap2();

    // Add inclined columns
    std::vector<Eigen::Vector2d> base_centers = {{6.5, 2.0}, {11.0, 6.0}};
    double size = 0.3;
    double height = 2.5;

    for (const auto& base : base_centers) {
        for (double z = 0; z <= height; z += 0.2) {
            double x_offset = 0.1 * z;
            for (double x = base.x() + x_offset - size / 2; x <= base.x() + x_offset + size / 2; x += 0.05) {
                for (double y = base.y() - size / 2; y <= base.y() + size / 2; y += 0.05) {
                    cloud->points.emplace_back(x, y, z);
                }
            }
        }
    }

    return cloud;
}

PointCloud::Ptr generateMap4() {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    double wall_x = 7.0;
    double door_y_min = 3.5, door_y_max = 4.5;
    double wall_height = 2.5;
    double resolution = 0.1;

    for (double y = 0; y <= 8.0; y += resolution) {
        if (y >= door_y_min && y <= door_y_max) continue;
        for (double z = 0; z <= wall_height; z += resolution) {
            cloud->points.emplace_back(wall_x, y, z);
        }
    }

    *cloud += *generateBoundary();
    return cloud;
}

PointCloud::Ptr generateMap5() {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    double wall_height = 2.5;
    double resolution = 0.1;

    for (double x = 5.0; x <= 10.0; x += resolution) {
        for (double z = 0; z <= wall_height; z += resolution) {
            for (double dy = -0.025; dy <= 0.025; dy += 0.025) {
                cloud->points.emplace_back(x, 3.25 + dy, z); // left wall
                cloud->points.emplace_back(x, 4.75 + dy, z); // right wall
            }
        }
    }

    *cloud += *generateBoundary();
    return cloud;
}

PointCloud::Ptr generateMapByID(int id) {
    switch (id) {
        case 1: return generateMap1();
        case 2: return generateMap2();
        case 3: return generateMap3();
        case 4: return generateMap4();
        case 5: return generateMap5();
        default:
            ROS_WARN("Unknown map_id: %d, defaulting to Map1", id);
            return generateMap1();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "exp_map");
    ros::NodeHandle nh("~");

    nh.param("map_id", map_id, 1);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/mock_map", 1);

    PointCloud::Ptr map_cloud = generateMapByID(map_id);
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header.frame_id = "world";

    ros::Rate rate(1.0);
    while (ros::ok()) {
        map_msg.header.stamp = ros::Time::now();
        map_pub.publish(map_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}