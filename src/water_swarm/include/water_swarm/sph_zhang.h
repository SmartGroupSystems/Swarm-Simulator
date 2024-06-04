#ifndef __SPH_ZHANG_H__
#define __SPH_ZHANG_H__

#define PI 3.14159265f

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>
#include <thread>
#include <math.h>

#include "water_swarm/Position.h"
#include "water_swarm/Velocity.h"
#include "water_swarm/Acceleration.h"
#include "water_swarm/Force.h"
#include "water_swarm/Odom.h"
#include "water_swarm/OdomWithNeighbors.h"
#include "water_swarm/OdomBroadcast.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "bspline_race/BsplineTraj.h"

ros::Timer                                              timer;
ros::Subscriber                                         nav_goal_sub;
ros::Publisher                                          particles_publisher;
ros::Publisher                                          virtual_particles_publisher;

ros::Time last_time;//控制时间loop
ros::Time last_print_time;//打印时间loop
ros::Time current_time;

int    particleCount;
double particleInterval;
double particleVisScale;
double updateInterval;
double threshold_dist;
float  mass, restDensity, h, g;
double  k_den, k_rep, k_fri;

void odomBroadcastCallback(const water_swarm::OdomBroadcast::ConstPtr& msg);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);

struct SPHSettings
{   
    // 添加的默认构造函数
    SPHSettings()
    : mass(1.0f),            // 默认质量
      restDensity(1.0f),     // 默认静止密度，水的密度大约是1000kg/m^3
      h(1.0f),               // 影响半径，决定粒子影响的范围
      g(-9.81f)              // 重力加速度，向下是负值
    {
        h2 = h * h;
        poly = 10 * 54.97 / ( 7 * PI * h2);
        selfDens = mass * poly ;
    };

    SPHSettings(
        float mass, float restDensity, float h, float g)
    : mass(mass)
    , restDensity(restDensity)
    , h(h)
    , g(g)
    {
        h2 = h * h;
        poly = 10 * 54.97 / ( 7 * PI * h2);
        selfDens = mass * poly ;
    }

    float poly, polyGrad, mass, h2, selfDens, restDensity,  h, g;
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
    water_swarm::Force          u_den, u_rep, u_fri;
    std_msgs::String            name;
};

class SPHSystem
{
public:
    SPHSettings     settings;
    size_t          sph_particleCubeWidth;
    size_t          sph_particleCount;

    bool started;
    bool runOnGPU;
    bool isInitialReceived = false;  // 用于检查是否已经接收到第一个消息

public:
	SPHSystem(
        size_t numParticles, const SPHSettings &settings,
        const bool &runOnGPU);
    SPHSystem();
	~SPHSystem();

    std::vector<Particle>       particles;
    std::vector<Particle>       virtual_particles;
    std::map<const Particle*, std::vector<std::pair<const Particle*, float>>> particleNeighborsTable;
    
    //initializes the particles that will be used
	void initParticles();
    void findNeighbors();
    
    // Finite State Machine
    // updates the SPH system
	void update(float deltaTime);
	void reset();
	void start();
    void stop();

    /// Update attrs of particles in place.
    void updateParticles(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime, const bool onGPU);

    /// Update attrs of particles in place.
    // Here I didn't write the GPU part now...
    void updateParticlesGPU(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime){};

    void updateParticlesCPU(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime);
    
    void calaDynamicBound();
    bool isNearVirtualParticle(double x, double y, double z);
    void generateVirtualParticles(const double l, const int particlesPerSide, const water_swarm::Position& apex);
    void parallelDensityAndPressures();
    void parallelForces();
    void parallelViscosity();
    void parallelUpdateParticlePositions(const float deltaTime);
    void pubroscmd();

};

#endif