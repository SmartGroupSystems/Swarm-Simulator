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
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    // 高低差柱子：低0.6m，高2.0m
    std::vector<std::pair<Eigen::Vector2d, double>> columns = {
        {{5.0, 2.0}, 0.6}, {{6.5, 4.0}, 2.0}, {{8.0, 6.0}, 0.6},
        {{9.5, 3.0}, 2.0}, {{11.0, 5.0}, 0.6}, {{12.5, 2.0}, 2.0}
    };
    double size = 0.3;

    for (const auto& c : columns) {
        for (double x = c.first.x() - size / 2; x <= c.first.x() + size / 2; x += 0.05) {
            for (double y = c.first.y() - size / 2; y <= c.first.y() + size / 2; y += 0.05) {
                for (double z = 0; z <= c.second; z += 0.2) {
                    cloud->points.emplace_back(x, y, z);
                }
            }
        }
    }

    // 斜柱连接高低柱子
    for (size_t i = 0; i + 1 < columns.size(); ++i) {
        Eigen::Vector3d start(columns[i].first.x(), columns[i].first.y(), columns[i].second);
        Eigen::Vector3d end(columns[i + 1].first.x(), columns[i + 1].first.y(), columns[i + 1].second);
        for (double t = 0.0; t <= 1.0; t += 0.05) {
            Eigen::Vector3d pt = (1 - t) * start + t * end;
            for (double dx = -0.05; dx <= 0.05; dx += 0.05) {
                for (double dy = -0.05; dy <= 0.05; dy += 0.05) {
                    cloud->points.emplace_back(pt.x() + dx, pt.y() + dy, pt.z());
                }
            }
        }
    }

    *cloud += *generateBoundary();
    return cloud;
}

PointCloud::Ptr generateMap4() {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "world";

    double wall_x = 7.0;
    double door_y_min = 3.0, door_y_max = 5.0;
    double wall_height = 3.0;
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

    // 通道墙宽：2m (y: 3.0 ~ 5.0)
    for (double x = 5.0; x <= 10.0; x += resolution) {
        for (double z = 0; z <= wall_height; z += resolution) {
            for (double dy = -0.025; dy <= 0.025; dy += 0.025) {
                cloud->points.emplace_back(x, 3.0 + dy, z);
                cloud->points.emplace_back(x, 5.0 + dy, z);
            }
        }
    }

    // 左门框（y=3.0）连接起始点
    for (double y = 0; y <= 3.0; y += resolution) {
        for (double z = 0; z <= wall_height; z += resolution) {
            cloud->points.emplace_back(5.0, y, z);
        }
    }

    // 右门框（y=5.0）连接起始点
    for (double y = 5.0; y <= 8.0; y += resolution) {
        for (double z = 0; z <= wall_height; z += resolution) {
            cloud->points.emplace_back(5.0, y, z);
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