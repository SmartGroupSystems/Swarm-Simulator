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
  Eigen::Vector3d camera_pos_;
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class gvf
{
    public:
        VectorFieldData gvf_;
        ros::Subscriber indep_odom_sub_, indep_cloud_sub_;
        ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
        ros::Timer esdf_timer_, vis_timer_;
        
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
        void publishMap();
        void publishMapInflate(bool all_info = false);
        void publishESDF();
        void publishUpdateRange();

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

}

#endif