#ifndef __SPH_H__
#define __SPH_H__

#define PI 3.14159265f

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>
#include <thread>

#include "common_msgs/Position.h"
#include "common_msgs/Velocity.h"
#include "common_msgs/Acceleration.h"
#include "common_msgs/Force.h"
#include "common_msgs/Odom.h"
#include "common_msgs/OdomWithNeighbors.h"
#include "common_msgs/OdomBroadcast.h"
#include "common_msgs/PositionCommand.h"
#include "common_msgs/BsplineTraj.h"

ros::Subscriber                                         odomBroadcast_sub;
ros::Timer                                              timer;
ros::Subscriber                                         nav_goal_sub;
ros::Publisher                                          particles_publisher;
ros::Publisher                                          virtual_particles_vis;
ros::Subscriber                                         traj_sub;

std::map<std::string, ros::Publisher>                   uav_publishers;
std::map<std::string, ros::Subscriber>                  odomSubscribers;
std::map<std::string, common_msgs::OdomWithNeighbors>   odomWithNeighbors;

//存储轨迹
std::vector<geometry_msgs::PoseStamped> global_positions;
std::vector<geometry_msgs::PoseStamped> global_velocities;
std::vector<geometry_msgs::PoseStamped> global_accelerations;

bool isInitialReceived = false;  // 用于检查是否已经接收到第一个odomBroadcast
common_msgs::OdomBroadcast  initial_odomBroadcast_;
common_msgs::OdomBroadcast  current_odomBroadcast_;

ros::Time last_time;//控制时间loop
ros::Time last_print_time;//打印时间loop
ros::Time current_time;

float mass, restDensity, gasConstant, viscosity, h, g, tension;

bool use_pctrl = false;//使用位置控制
bool use_vctrl = false;//使用速度控制
bool use_actrl = false;//使用加速度控制
bool first_traj= true;//第一次接收轨迹

double particlessideLength; // 边界边长
int    particlesPerSide;    // 每边x个粒子

void odomBroadcastCallback(const common_msgs::OdomBroadcast::ConstPtr& msg);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void subscribeOdomWithNeighbors(const std::string &topic_name, ros::NodeHandle &nh);
void odomWithNeighborsCallback(const common_msgs::OdomWithNeighborsConstPtr& msg, const std::string& uav_name); 
void trajCallback(const common_msgs::BsplineTraj::ConstPtr& msg);
void publishPositionCommand(const std::string& uav_name, ros::NodeHandle& nh);

struct SPHSettings
{   
    // 添加的默认构造函数
    // 0.02f, 1000, 1, 1.04f, 0.15f, -9.8f, 0.2f
    SPHSettings()
    : mass(1.0f),            // 默认质量
      restDensity(1000.0f),  // 默认静止密度，水的密度大约是1000kg/m^3
      gasConstant(2000.0f),     // 气体常数，这个值取决于模拟的流体类型和效果
      viscosity(3.5f),       // 粘度，决定流体的粘滞特性
      h(1.0f),               // 影响半径，决定粒子影响的范围
      g(-9.81f),             // 重力加速度，向下是负值
      tension(0.0728f)      // 表面张力，水的表面张力
    {
        poly6 = 315.0f / (64.0f * PI * pow(h, 9));
        spikyGrad = -45.0f / (PI * pow(h, 6));
        spikyLap = 45.0f / (PI * pow(h, 6));
        h2 = h * h;
        selfDens = mass * poly6 * pow(h, 6);
        massPoly6Product = mass * poly6;
    };

    SPHSettings(
        float mass, float restDensity, float gasConst, float viscosity,
        float h, float g, float tension)
    : mass(mass)
    , restDensity(restDensity)
    , gasConstant(gasConst)
    , viscosity(viscosity)
    , h(h)
    , g(g)
    , tension(tension)
    {
    poly6 = 315.0f / (64.0f * PI * pow(h, 9));
    spikyGrad = -45.0f / (PI * pow(h, 6));
    spikyLap = 45.0f / (PI * pow(h, 6));
    h2 = h * h;
    selfDens = mass * poly6 * pow(h, 6);
    massPoly6Product = mass * poly6;
    }

    float poly6, spikyGrad, spikyLap, gasConstant, mass, h2, selfDens,
        restDensity, viscosity, h, g, tension, massPoly6Product;
};

struct Particle
{
    common_msgs::Position       position;
    common_msgs::Velocity       velocity;
    common_msgs::Acceleration   acceleration;
    common_msgs::Force          force;
    float                       density;
    float                       pressure;
    uint16_t                    hash;
    std_msgs::String            name;
};

class SPHSystem
{
public:
    SPHSettings     settings;
    size_t          particleCubeWidth;

    bool started;
    bool runOnGPU;
    bool isInitialReceived = false;  // 用于检查是否已经接收到第一个消息

    //initializes the particles that will be used
	void initParticles();

public:
	SPHSystem(
        size_t numParticles, const SPHSettings &settings,
        const bool &runOnGPU);
    SPHSystem();
	~SPHSystem();

    Particle                    *particles;
    size_t                      particleCount;
    std::vector<Particle>       virtual_particles;

    /// Finite State Machine
    //updates the SPH system
	void update(float deltaTime);
	void reset();
	void start();
    void stop();

    /// init planner
    void initPlanner();

    /// Update attrs of particles in place.
    void updateParticles(
        Particle *particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime, const bool onGPU);

    /// Update attrs of particles in place.
    // Here I didn't write the GPU part now...
    void updateParticlesGPU(
        Particle *particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime){};

    void updateParticlesCPU(
        Particle *particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime);
    
    void calaDynamicBound();
    bool isNearVirtualParticle(double x, double y, double z);
    void generateVirtualParticles(const double l, const int particlesPerSide, const common_msgs::Position& apex);
    void parallelDensityAndPressures();
    void parallelForces();
    void parallelUpdateParticlePositions(const float deltaTime);
    void pubroscmd();

};

// Function to extract the UAV number from the input string
inline std::string extractUavName(const std::string& uav_name) {
    std::regex pattern("/uav\\d+"); // Regular expression to match /uav followed by one or more digits
    std::smatch match;

    // Search for the pattern in the input string
    if (std::regex_search(uav_name, match, pattern)) {
        return match.str(0); // Return the matched string
    }
    return ""; // Return empty string if no match is found
}

#endif