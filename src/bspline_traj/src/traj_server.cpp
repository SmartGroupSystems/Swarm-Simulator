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
{}


void traj_cb(const common_msgs::BsplineTrajConstPtr &msg)
{}


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