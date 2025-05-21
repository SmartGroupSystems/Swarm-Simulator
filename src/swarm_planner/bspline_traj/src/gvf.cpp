#include "bspline_race/gvf.h"

namespace FLAG_Race
{
void gvf::init(ros::NodeHandle& nh, const std::string& particle, const std::string& odom, const std::string& cloud)
{
    // === 地图参数获取 ===
    double x_size, y_size, z_size;
    nh.param("sdf_map/resolution", gvf_.resolution_, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/local_update_range_x", gvf_.local_update_range_(0), -1.0);
    nh.param("sdf_map/local_update_range_y", gvf_.local_update_range_(1), -1.0);
    nh.param("sdf_map/local_update_range_z", gvf_.local_update_range_(2), -1.0);
    nh.param("sdf_map/obstacles_inflation", gvf_.obstacles_inflation_, -1.0);

    nh.param("sdf_map/esdf_slice_height", gvf_.esdf_slice_height_, -0.1);
    nh.param("sdf_map/visualization_truncate_height", gvf_.visualization_truncate_height_, -0.1);
    nh.param("sdf_map/virtual_ceil_height", gvf_.virtual_ceil_height_, -0.1);
    nh.param("sdf_map/ground_height", gvf_.ground_height_, 1.0);

    nh.param("sdf_map/show_occ_time", gvf_.show_occ_time_, false);
    nh.param("sdf_map/show_esdf_time", gvf_.show_esdf_time_, false);
    nh.param("sdf_map/frame_id", gvf_.frame_id_, std::string("world"));
    nh.param("sdf_map/local_bound_inflate", gvf_.local_bound_inflate_, 1.0);
    nh.param("sdf_map/local_map_margin", gvf_.local_map_margin_, 1);

    gvf_.local_bound_inflate_ = std::max(gvf_.resolution_, gvf_.local_bound_inflate_);
    gvf_.resolution_inv_ = 1.0 / gvf_.resolution_;
    gvf_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, gvf_.ground_height_);
    gvf_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    for (int i = 0; i < 3; ++i)
        gvf_.map_voxel_num_(i) = std::ceil(gvf_.map_size_(i) / gvf_.resolution_);

    gvf_.map_min_boundary_ = gvf_.map_origin_;
    gvf_.map_max_boundary_ = gvf_.map_origin_ + gvf_.map_size_;

    gvf_.map_min_idx_ = Eigen::Vector3i::Zero();
    gvf_.map_max_idx_ = gvf_.map_voxel_num_ - Eigen::Vector3i::Ones();

    // === 初始化数据缓存 ===
    int buffer_size = gvf_.map_voxel_num_(0) *
                      gvf_.map_voxel_num_(1) *
                      gvf_.map_voxel_num_(2);

    gvf_.occupancy_buffer_         = std::vector<double>(buffer_size, 0.0);
    gvf_.occupancy_buffer_neg      = std::vector<char>(buffer_size, 0);
    gvf_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);

    gvf_.distance_buffer_      = std::vector<double>(buffer_size, 10000.0);
    gvf_.distance_buffer_neg_  = std::vector<double>(buffer_size, 10000.0);
    gvf_.distance_buffer_all_  = std::vector<double>(buffer_size, 10000.0);

    gvf_.tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);
    gvf_.tmp_buffer2_ = std::vector<double>(buffer_size, 0.0);

    // === 初始化局部更新区域和统计量 ===
    gvf_.local_bound_min_ = Eigen::Vector3i::Zero();
    gvf_.local_bound_max_ = Eigen::Vector3i::Zero();

    gvf_.esdf_time_ = 0.0;
    gvf_.max_fuse_time_ = gvf_.max_esdf_time_ = 0.0;
    gvf_.update_num_ = 0;

    gvf_.local_updated_ = false;
    gvf_.esdf_need_update_ = false;
    gvf_.has_odom_ = false;

    gvf_.esdf_time_ = 0.0;
    gvf_.update_num_ = 0;
    gvf_.max_esdf_time_ = 0.0;
    gvf_.max_fuse_time_ = 0.0;

    esdf_timer_ = nh.createTimer(ros::Duration(0.05), &gvf::updateESDFCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.10), &gvf::visCallback, this);

    indep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom, 10, &gvf::odomCallback, this);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"gvf/occupancy", 10);
    map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"gvf/occupancy_inflate", 10);
    esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"gvf/esdf", 10);
    update_range_pub_ = nh.advertise<visualization_msgs::Marker>(particle +"gvf/update_range", 10);

}

template <typename F_get_val, typename F_set_val>
void gvf::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) 
{
    int v[gvf_.map_voxel_num_(dim)];
    double z[gvf_.map_voxel_num_(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++) {
        k++;
        double s;

        do {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++) {
        while (z[k + 1] < q) k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
        }
}

void gvf::updateESDF3d() 
{
    Eigen::Vector3i min_esdf = gvf_.local_bound_min_;
    Eigen::Vector3i max_esdf = gvf_.local_bound_max_;

    // ========== compute positive DT ==========

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int z, double val) { gvf_.tmp_buffer1_[toAddress(x, y, z)] = val; }, 
                min_esdf[2], max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return gvf_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { gvf_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
                min_esdf[1], max_esdf[1], 1);
        }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return gvf_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    gvf_.distance_buffer_[toAddress(x, y, z)] = 
                        gvf_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    // ========== compute negative distance ==========

    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                if (gvf_.occupancy_buffer_inflate_[idx] == 0) {
                    gvf_.occupancy_buffer_neg[idx] = 1;
                } else if (gvf_.occupancy_buffer_inflate_[idx] == 1) {
                    gvf_.occupancy_buffer_neg[idx] = 0;
                } else {
                    ROS_ERROR("what?");
                }
            }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return gvf_.occupancy_buffer_neg[
                        x * gvf_.map_voxel_num_[1] * gvf_.map_voxel_num_[2] +
                        y * gvf_.map_voxel_num_[2] + z] == 1 ?
                        0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val) { gvf_.tmp_buffer1_[toAddress(x, y, z)] = val; },
                min_esdf[2], max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return gvf_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { gvf_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
                min_esdf[1], max_esdf[1], 1);
        }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return gvf_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    gvf_.distance_buffer_neg_[toAddress(x, y, z)] = 
                        gvf_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    // ========== combine pos and neg DT ==========

    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                gvf_.distance_buffer_all_[idx] = gvf_.distance_buffer_[idx];
                if (gvf_.distance_buffer_neg_[idx] > 0.0)
                    gvf_.distance_buffer_all_[idx] += 
                        (-gvf_.distance_buffer_neg_[idx] + gvf_.resolution_);
            }
}

void gvf::odomCallback(const nav_msgs::OdometryConstPtr& odom) 
{
  gvf_.camera_pos_(0) = odom->pose.pose.position.x;
  gvf_.camera_pos_(1) = odom->pose.pose.position.y;
  gvf_.camera_pos_(2) = odom->pose.pose.position.z;

  gvf_.has_odom_ = true;
}

void gvf::resetBuffer() {
  Eigen::Vector3d min_pos = gvf_.map_min_boundary_;
  Eigen::Vector3d max_pos = gvf_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  gvf_.local_bound_min_ = Eigen::Vector3i::Zero();
  gvf_.local_bound_max_ = gvf_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void gvf::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        gvf_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

void gvf::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!gvf_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  gvf_.esdf_time_ += (t2 - t1).toSec();
  gvf_.max_esdf_time_ = max(gvf_.max_esdf_time_, (t2 - t1).toSec());

  if (gvf_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             gvf_.esdf_time_ / gvf_.update_num_, gvf_.max_esdf_time_);

  gvf_.esdf_need_update_ = false;
}

void gvf::visCallback(const ros::TimerEvent& /*event*/) {
  //publishMap();
  publishMapInflate(false);
  publishUpdateRange();
  publishESDF();

}

void gvf::publishMap() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_;
  Eigen::Vector3i max_cut = gvf_.local_bound_max_;

  int lmm = gvf_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > gvf_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void gvf::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_;
  Eigen::Vector3i max_cut = gvf_.local_bound_max_;

  if (all_info) {
    int lmm = gvf_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > gvf_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void gvf::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(gvf_.local_bound_min_, esdf_min_pos);
  indexToPos(gvf_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = gvf_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void gvf::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_ -
      Eigen::Vector3i(gvf_.local_map_margin_, gvf_.local_map_margin_, gvf_.local_map_margin_);
  Eigen::Vector3i max_cut = gvf_.local_bound_max_ +
      Eigen::Vector3i(gvf_.local_map_margin_, gvf_.local_map_margin_, gvf_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = gvf_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

}