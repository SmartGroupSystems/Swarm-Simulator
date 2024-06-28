#include "bspline_race/bspline_race.h"

using namespace std;
// bspline
bool first_bs = true;
#define PI acos(-1)
#define T_RATE 20.0
double set_height = 0.8;//这个参数后期可以修改
double roll, pitch, yaw;//定义存储r\p\y的容器
double last_yaw;
double last_yaw_dot;

common_msgs::PositionCommand pva_msg;
nav_msgs::Path vis_path;
class bs_traj
{
    public:
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d acc_;
        Eigen::Vector3d jerk_;
        int seq_;
};
std::vector<bs_traj> BTraj;
int current_pub_seq = 0;
double delta_T = 0.02;
double D_YAW_MAX = PI/2;
double output_yaw;
double output_d_yaw;
double YAW_MAX = D_YAW_MAX * delta_T;
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw);
std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw);

ros::Publisher  cmd_pub;
ros::Subscriber cmd_sub;

void run()
{
  if(BTraj.size() != 0)
    {
      bs_traj BT_ptr = *(BTraj.begin());
      BTraj.erase(BTraj.begin());

      // CAL_YAW
      double arg_    = atan2(-BT_ptr.vel_[0],BT_ptr.vel_[1]) + (PI/2.0f);
      double vel_len = sqrt(pow(BT_ptr.vel_[0],2)+pow(BT_ptr.vel_[1],2));
      if(vel_len<=0.1) arg_ = last_yaw;
      std::pair<double, double> yaw_all = calculate_yaw(last_yaw,arg_);
      geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(yaw_all.first);

      // POSE
      pva_msg.position.x = BT_ptr.pos_[0];
      pva_msg.position.y = BT_ptr.pos_[1];
      pva_msg.position.z = set_height;
      // VEL
      // pva_msg.velocity.x = BT_ptr.vel_[0];
      // pva_msg.velocity.y = BT_ptr.vel_[1];
      // pva_msg.velocity.z = 0;
      // // ACC
      // pva_msg.acceleration.x = BT_ptr.acc_[0];
      // pva_msg.acceleration.y = BT_ptr.acc_[1];
      // pva_msg.acceleration.z = 0;
      // //JERK
      // pva_msg.jerk.x = BT_ptr.jerk_[0];
      // pva_msg.jerk.y = BT_ptr.jerk_[1];
      // pva_msg.jerk.z = 0;
      // YAW
      pva_msg.yaw      = yaw_all.first;
      pva_msg.yaw_dot  = yaw_all.second;

      // TIME
      pva_msg.header.stamp = ros::Time::now();
      cmd_pub.publish(pva_msg);
      cout<<"[cmd] Pub traj [x y z vx vy vz]: " <<   pva_msg.position.x  << "  "
                                                <<   pva_msg.position.y  << "  "
                                                <<   pva_msg.position.z  << "  "
                                                <<   pva_msg.velocity.x  << "  "
                                                <<   pva_msg.velocity.y  << "  "                                                
                                                <<   pva_msg.velocity.z  << "  "                                       
                                                << endl;
    }

  else
    {
      cout <<"[cmd] Arrived!"<< endl;
    }
}


void traj_cb(const common_msgs::BsplineTrajConstPtr &msg)
{
  ROS_INFO("RECEIVED TRAJ, SIZE: %d",msg->position.size());
  // 收到新轨迹
  if(first_bs)
  {
    // 直接放到BTraj里
    for (size_t i = 0; i < msg->position.size(); i++)
    {    
        bs_traj BT_ptr;
        BT_ptr.pos_ << msg->position[i].pose.position.x, 
                       msg->position[i].pose.position.y,
                       msg->position[i].pose.position.z;
        BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
                       msg->velocity[i].pose.position.y,
                       msg->velocity[i].pose.position.z;
        BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
                       msg->acceleration[i].pose.position.y,
                       msg->acceleration[i].pose.position.z;           
        BT_ptr.jerk_<< msg->jerk[i].pose.position.x,
                       msg->jerk[i].pose.position.y,
                       msg->jerk[i].pose.position.z;      

        BTraj.push_back(BT_ptr);
    }

    first_bs = false;
  }
}


std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw)
{
    std::pair<double, double> yaw_yawdot(0, 0);
    if(current_yaw<0)                 current_yaw = current_yaw + 2*PI;
    else if(current_yaw>2*PI)  current_yaw = current_yaw - 2*PI;
      if(aim_yaw<0)                 aim_yaw = aim_yaw + 2*PI;
    else if(aim_yaw>2*PI)    aim_yaw = aim_yaw - 2*PI;
    double yaw_distance = aim_yaw - current_yaw;
    double sign_        = yaw_distance / fabs(yaw_distance);
    if(fabs(yaw_distance) < YAW_MAX )
    {cout<<"ca1"<<endl;
      output_yaw   = aim_yaw;
      output_d_yaw = yaw_distance / delta_T;
    }
    else
    {cout<<"ca2"<<endl;
      output_yaw = current_yaw + sign_ * YAW_MAX;
      output_d_yaw = sign_*D_YAW_MAX;
    }
    yaw_yawdot.first = output_yaw;
    yaw_yawdot.second = output_d_yaw;
    return yaw_yawdot;
}

std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw)
{
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw_ = 0;
    double yawdot = 0;
    if (aim_yaw - current_yaw > PI)
    {
      
      if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
      {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
          yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
      }
      else
      {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
          yawdot = -D_YAW_MAX;
        else
          yawdot = (aim_yaw - current_yaw) /delta_T;
      }
    }
    else if (aim_yaw - current_yaw < -PI)
    {
      if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
      {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
          yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
      }
      else
      {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw < -PI)
          yawdot = D_YAW_MAX;
        else
          yawdot = (aim_yaw - current_yaw) /delta_T;
      }
    }
    else
    {
      if (aim_yaw - current_yaw < -YAW_MAX)
      {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
          yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
      }
      else if (aim_yaw - current_yaw > YAW_MAX)
      {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
          yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
      }
      else
      {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
          yawdot = -D_YAW_MAX;
        else if (yaw_ - current_yaw < -PI)
          yawdot = D_YAW_MAX;
        else
          yawdot = (aim_yaw - current_yaw) /delta_T;
      }
    }
      if (fabs(yaw_ - last_yaw) <= YAW_MAX)
      yaw = 0.5 * last_yaw + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
    last_yaw = yaw_;  
    last_yaw_dot = yawdot;
    yaw_yawdot.first = yaw_;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle nh("~");
    cmd_pub   = nh.advertise<common_msgs::PositionCommand>("/position_cmd", 10, true);
    cmd_sub   = nh.subscribe("/bspline_traj",1, traj_cb);
    ros::Rate rate(T_RATE);

while(ros::ok())
    {
      run();
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}