#ifndef  _GVF_H
#define  _GVF_H   

//standard
#include <fstream>
#include <string>
#include <regex>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <numeric>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;

namespace FLAG_Race
{
struct VectorFieldData {
  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_min_idx_, map_max_idx_;
  Eigen::Vector3d local_update_range_;
  Eigen::Vector3d camera_pos_, last_camera_pos_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;

  /* local map update and clear */
  double local_bound_inflate_;
  int local_map_margin_;

  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_neg;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;  

  /* visualization and computation time display */
  double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;

  bool show_esdf_time_, show_occ_time_, has_odom_,local_updated_, esdf_need_update_;
  // range of updating ESDF
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  // computation time
  double esdf_time_, max_fuse_time_, max_esdf_time_;

  int update_num_;
  
  //GVF GAIN
  double K1_, K2_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class gvf
{
    public:
        VectorFieldData gvf_;
        ros::Subscriber indep_odom_sub_, indep_cloud_sub_, path_sub_, goal_sub_, kino_path_sub_;
        ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_,gvf_vis_pub_;
        ros::Timer esdf_timer_, vis_timer_;
        nav_msgs::Path last_path_;
        bool use_kinopath_;
        bool use_quad_fit_;  // 添加新参数

    public:
        gvf(){};  
        ~gvf(){};
        void init(ros::NodeHandle& nh,const std::string& particle, const std::string& odom, const std::string& cloud);
        template <typename F_get_val, typename F_set_val>
            void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        void updateESDF3d();
        void odomCallback(const nav_msgs::OdometryConstPtr& odom);
        void resetBuffer();
        void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
        void updateESDFCallback(const ros::TimerEvent& /*event*/);
        void visCallback(const ros::TimerEvent& /*event*/);
        void pathCallback(const nav_msgs::Path::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void kinoPathCallback(const nav_msgs::Path::ConstPtr& msg);
        void publishMap();
        void publishMapInflate(bool all_info = false);
        void publishESDF();
        void publishGVF();
        void publishUpdateRange();
        Eigen::Vector3d calcGuidingVectorField2D(const Eigen::Vector3d pos);
        Eigen::Vector3d calcGuidingVectorField3D(const Eigen::Vector3d pos);
        Eigen::Vector3d estimateTangentViaQuadraticFit(const Eigen::Vector3d& pos);
        
        inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
        inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
        inline int toAddress(const Eigen::Vector3i& id);
        inline int toAddress(int& x, int& y, int& z);
        inline bool isInMap(const Eigen::Vector3d& pos);
        inline bool isInMap(const Eigen::Vector3i& idx);
        inline bool isInBox(const Eigen::Vector3i& id);
        inline bool isInBox(const Eigen::Vector3d& pos);
        inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
        inline void setOccupied(Eigen::Vector3d pos);
        inline int getOccupancy(Eigen::Vector3d pos);
        inline int getOccupancy(Eigen::Vector3i id);
        inline int getInflateOccupancy(Eigen::Vector3d pos);
        inline bool getNearestFreePoint(const Eigen::Vector3d& pos, Eigen::Vector3d& near_pos);
        inline void boundIndex(Eigen::Vector3i& id);
        inline bool isUnknown(const Eigen::Vector3i& id);
        inline bool isUnknown(const Eigen::Vector3d& pos);
        inline bool isKnownFree(const Eigen::Vector3i& id);
        inline bool isKnownOccupied(const Eigen::Vector3i& id);
        inline double getDistance(const Eigen::Vector3d& pos);
        inline double getDistance(const Eigen::Vector3i& id);
        inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
        inline double getDistWithGradQuadraticFit(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
        double getGain() const { return gain_; }  // 获取GVF增益
        
    private:
        double gain_;  // GVF增益
};

/* ============================== definition of inline function
 * ============================== */

inline int gvf::toAddress(const Eigen::Vector3i& id) {
  return id(0) * gvf_.map_voxel_num_(1) * gvf_.map_voxel_num_(2) + id(1) * gvf_.map_voxel_num_(2) + id(2);
}

inline int gvf::toAddress(int& x, int& y, int& z) {
  return x * gvf_.map_voxel_num_(1) * gvf_.map_voxel_num_(2) + y * gvf_.map_voxel_num_(2) + z;
}

inline void gvf::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), gvf_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), gvf_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), gvf_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline void gvf::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - gvf_.map_origin_(i)) * gvf_.resolution_inv_);
}

inline void gvf::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * gvf_.resolution_ + gvf_.map_origin_(i);
}

inline double gvf::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return gvf_.distance_buffer_all_[toAddress(id)];
}

inline double gvf::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return gvf_.distance_buffer_all_[toAddress(id1)];
}

inline bool gvf::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < gvf_.map_min_boundary_(0) + 1e-4 || pos(1) < gvf_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < gvf_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > gvf_.map_max_boundary_(0) - 1e-4 || pos(1) > gvf_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > gvf_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool gvf::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > gvf_.map_voxel_num_(0) - 1 || idx(1) > gvf_.map_voxel_num_(1) - 1 ||
      idx(2) > gvf_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline double gvf::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * gvf_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * gvf_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * gvf_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * gvf_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= gvf_.resolution_inv_;

  if ( dist < 0) dist = 0.0;

  return dist;
}

inline double gvf::getDistWithGradQuadraticFit(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0.0;
  }

  // 获取局部邻域点（如 5x5x5 网格），用于拟合
  std::vector<Eigen::Vector3d> neighbor_pts;
  std::vector<double> neighbor_vals;

  const int N = 2;  // 拟合窗口半径，即使用 (2N+1)^3 个点
  Eigen::Vector3i center_idx;
  posToIndex(pos, center_idx);
  Eigen::Vector3d center_pos;
  indexToPos(center_idx, center_pos);

  // 检查是否存在未初始化的点（距离值为10000）
  bool has_uninitialized = false;
  double max_valid_dist = 0.0;

  for (int dx = -N; dx <= N; ++dx) {
    for (int dy = -N; dy <= N; ++dy) {
      for (int dz = -N; dz <= N; ++dz) {
        Eigen::Vector3i idx = center_idx + Eigen::Vector3i(dx, dy, dz);
        if (!isInMap(idx)) continue;

        Eigen::Vector3d pt;
        indexToPos(idx, pt);
        double val = getDistance(idx);

        // 检查是否为未初始化的点
        if (val > 9999.0) {
          has_uninitialized = true;
          continue;
        }

        max_valid_dist = std::max(max_valid_dist, val);
        neighbor_pts.push_back(pt);
        neighbor_vals.push_back(val);
      }
    }
  }

  // 如果存在未初始化的点，使用更保守的梯度估计
  if (has_uninitialized || neighbor_pts.size() < 10) {  // 至少需要10个点进行二阶拟合
    // 使用中心差分计算梯度
    const double eps = gvf_.resolution_;
    Eigen::Vector3d pos_eps;
    double dist_center = getDistance(pos);
    
    grad.setZero();
    for (int i = 0; i < 3; ++i) {
      pos_eps = pos;
      pos_eps[i] += eps;
      double dist_plus = getDistance(pos_eps);
      
      pos_eps = pos;
      pos_eps[i] -= eps;
      double dist_minus = getDistance(pos_eps);
      
      grad[i] = (dist_plus - dist_minus) / (2.0 * eps);
    }
    
    // 如果梯度太大，进行归一化
    double grad_norm = grad.norm();
    if (grad_norm > 1.0) {
      grad = grad / grad_norm;
    }
    
    return dist_center;
  }

  // 如果所有点都已初始化，使用二阶拟合
  int num_samples = neighbor_pts.size();
  Eigen::MatrixXd Phi(num_samples, 10);
  Eigen::VectorXd d(num_samples);

  for (int i = 0; i < num_samples; ++i) {
    Eigen::Vector3d delta = neighbor_pts[i] - center_pos;
    double dx = delta[0], dy = delta[1], dz = delta[2];

    Phi.row(i) << 1, dx, dy, dz,
                  0.5 * dx * dx, dx * dy, dx * dz,
                  0.5 * dy * dy, dy * dz,
                  0.5 * dz * dz;

    d(i) = neighbor_vals[i];
  }

  // 最小二乘拟合
  Eigen::VectorXd theta = (Phi.transpose() * Phi).ldlt().solve(Phi.transpose() * d);

  // 距离估计
  Eigen::Vector3d local_delta = pos - center_pos;
  double dx = local_delta[0], dy = local_delta[1], dz = local_delta[2];
  double dist = theta(0) + theta(1)*dx + theta(2)*dy + theta(3)*dz
                + 0.5*theta(4)*dx*dx + theta(5)*dx*dy + theta(6)*dx*dz
                + 0.5*theta(7)*dy*dy + theta(8)*dy*dz + 0.5*theta(9)*dz*dz;

  // 梯度估计（连续）
  grad[0] = theta(1) + theta(4)*dx + theta(5)*dy + theta(6)*dz;
  grad[1] = theta(2) + theta(5)*dx + theta(7)*dy + theta(8)*dz;
  grad[2] = theta(3) + theta(6)*dx + theta(8)*dy + theta(9)*dz;

  // 如果梯度太大，进行归一化
  double grad_norm = grad.norm();
  if (grad_norm > 1.0) {
    grad = grad / grad_norm;
  }

  if (dist < 0.0) dist = 0.0;

  return dist;
}

}

#endif