#ifndef __SPH_H__
#define __SPH_H__

#define PI 3.14159265f

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>
#include <thread>

#include "water_swarm/Position.h"
#include "water_swarm/Velocity.h"
#include "water_swarm/Acceleration.h"
#include "water_swarm/Force.h"
#include "water_swarm/Odom.h"
#include "water_swarm/OdomWithNeighbors.h"
#include "water_swarm/OdomBroadcast.h"

ros::Subscriber                                         odomBroadcast_sub;
ros::Timer                                              timer;
ros::Subscriber                                         nav_goal_sub;


std::map<std::string, ros::Subscriber>                  odomSubscribers;
std::map<std::string, water_swarm::OdomWithNeighbors>   odomWithNeighbors;

bool isInitialReceived = false;  // 用于检查是否已经接收到第一个odomBroadcast
water_swarm::OdomBroadcast  initial_odomBroadcast_;
water_swarm::OdomBroadcast  current_odomBroadcast_;

void odomBroadcastCallback(const water_swarm::OdomBroadcast::ConstPtr& msg);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void subscribeOdomWithNeighbors(const std::string &topic_name, ros::NodeHandle &nh);
void odomWithNeighborsCallback(const water_swarm::OdomWithNeighborsConstPtr& msg, const std::string& uav_name); 

struct SPHSettings
{   
    // 添加的默认构造函数
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
    water_swarm::Position       position;
    water_swarm::Velocity       velocity;
    water_swarm::Acceleration   acceleration;
    water_swarm::Force          force;
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
    void updateParticlesGPU(
        Particle *particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime){};

    void updateParticlesCPU(
        Particle *particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime);
    
    void parallelDensityAndPressures();
    void parallelForces();
    void parallelUpdateParticlePositions();
};

#endif