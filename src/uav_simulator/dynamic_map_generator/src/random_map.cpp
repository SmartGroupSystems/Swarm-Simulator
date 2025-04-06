#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>
#include <random>

ros::Publisher map_pub;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

std::random_device rd;
std::default_random_engine eng(rd());

// Parameters
double x_size, y_size, z_size, resolution;
int obs_num, circle_num;
double w_l, w_h, h_l, h_h;
double radius_l, radius_h, z_l, z_h, theta;
double init_x, init_y;
int circle_thickness;

void RandomMapGenerate() {
  cloudMap.clear();
  pcl::PointXYZ pt;

  std::uniform_real_distribution<double> rand_x(-x_size / 2.0, x_size / 2.0);
  std::uniform_real_distribution<double> rand_y(-y_size / 2.0, y_size / 2.0);
  std::uniform_real_distribution<double> rand_w(w_l, w_h);
  std::uniform_real_distribution<double> rand_h(h_l, h_h);
  std::uniform_real_distribution<double> rand_radius(radius_l, radius_h);
  std::uniform_real_distribution<double> rand_theta(-theta, theta);
  std::uniform_real_distribution<double> rand_z(z_l, z_h);

  // Generate square obstacles
  for (int i = 0; i < obs_num; ++i) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double w = rand_w(eng);
    double h = rand_h(eng);

    if (hypot(x - init_x, y - init_y) < 2.0) {
      --i; continue;
    }

    int widNum = ceil(w / resolution);
    int heiNum = ceil(h / resolution);

    for (int r = -widNum / 2; r <= widNum / 2; ++r) {
      for (int s = -widNum / 2; s <= widNum / 2; ++s) {
        for (int t = 0; t <= heiNum; ++t) {
          pt.x = x + r * resolution;
          pt.y = y + s * resolution;
          pt.z = t * resolution;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }

  // Generate vertical circular obstacles with thickness
  for (int i = 0; i < circle_num; ++i) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double z = rand_z(eng);

    if (hypot(x - init_x, y - init_y) < 2.0) {
      --i; continue;
    }

    Eigen::Vector3d center(x, y, z);
    Eigen::Matrix3d rotation;
    double angle = rand_theta(eng);
    rotation << 1, 0, 0,
                0, cos(angle), -sin(angle),
                0, sin(angle), cos(angle);

    double radius = rand_radius(eng);

    for (int thick = -circle_thickness; thick <= circle_thickness; ++thick) {
      double current_radius = radius + thick * resolution;
      for (double ang = 0; ang < 2 * M_PI; ang += resolution / current_radius) {
        Eigen::Vector3d point(0, current_radius * cos(ang), current_radius * sin(ang));
        point = rotation * point + center;
        pt.x = point(0);
        pt.y = point(1);
        pt.z = point(2);
        cloudMap.points.push_back(pt);
      }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_node");
  ros::NodeHandle nh("~");

  nh.param("x_size", x_size, 50.0);
  nh.param("y_size", y_size, 50.0);
  nh.param("z_size", z_size, 5.0);
  nh.param("resolution", resolution, 0.1);
  nh.param("obs_num", obs_num, 30);
  nh.param("circle_num", circle_num, 30);
  nh.param("circle_thickness", circle_thickness, 2);

  nh.param("w_l", w_l, 0.3);
  nh.param("w_h", w_h, 0.8);
  nh.param("h_l", h_l, 3.0);
  nh.param("h_h", h_h, 7.0);

  nh.param("radius_l", radius_l, 1.0);
  nh.param("radius_h", radius_h, 2.0);
  nh.param("z_l", z_l, 1.0);
  nh.param("z_h", z_h, 2.0);
  nh.param("theta", theta, M_PI);

  nh.param("init_x", init_x, 0.0);
  nh.param("init_y", init_y, 0.0);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/mock_map", 1, true);

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(cloudMap, map_msg);
  map_msg.header.frame_id = "world";

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    map_msg.header.stamp = ros::Time::now();
    map_pub.publish(map_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
