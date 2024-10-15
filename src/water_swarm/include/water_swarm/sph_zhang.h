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
#include <unordered_map>

#include "common_msgs/Position.h"
#include "common_msgs/Velocity.h"
#include "common_msgs/Acceleration.h"
#include "common_msgs/Force.h"
#include "common_msgs/Odom.h"
#include "common_msgs/OdomWithNeighbors.h"
#include "common_msgs/OdomBroadcast.h"
#include "common_msgs/Particle.h"
#include "common_msgs/Swarm_particles.h"
#include "common_msgs/PositionCommand.h"
#include "common_msgs/BsplineTraj.h"
#include "common_msgs/Swarm_traj.h"


ros::Timer                                              timer;
ros::Publisher                                          particles_publisher;
ros::Publisher                                          virtual_particles_publisher;
ros::Publisher                                          swarm_pub;
ros::Subscriber                                         swarm_traj_sub;
ros::Subscriber                                         target_sub;
ros::Subscriber                                         force_sub;
std::vector<ros::Publisher>                             odom_publishers;

ros::Time last_time;//控制时间loop
ros::Time last_print_time;//打印时间loop
ros::Time current_time;

int    particleCount;
double particleInterval;
double particleVisScale;
double updateInterval;
double threshold_dist;
float  mass, restDensity, h, g;
double k_den, k_rep, k_fri;
double r_1,r_2;
double v_max, a_max;

void odomBroadcastCallback(const common_msgs::OdomBroadcast::ConstPtr& msg);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void swarmTrajCallback(const common_msgs::Swarm_traj::ConstPtr& msg);
void targetCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
void forceCallback(const common_msgs::Swarm_particles::ConstPtr& msg);

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

enum ParticleState {
    NULL_STATE,  // "NULL" 状态
    TRAJ,        // "TRAJ" 状态
    NEED_TRAJ,   // 需要重规划
    ATTRACT,     // "吸引" 状态，对应英文 "attract"
    REPEL        // "排斥" 状态，对应英文 "repel"
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
    common_msgs::Force          u_den, u_rep, u_fri;
    std_msgs::String            name;
    int                         index;
    ParticleState               state = NULL_STATE;
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

    std::unordered_map<int, common_msgs::BsplineTraj> swarmTrajBuffer_;
    std::unordered_map<int, common_msgs::Position> targetMap;
    std::unordered_map<int, common_msgs::Force> forceMap;

public:
	SPHSystem(
        size_t numParticles, const SPHSettings &settings,
        const bool &runOnGPU);
    SPHSystem();
	~SPHSystem();

    std::vector<Particle>       particles;
    std::vector<Particle>       virtual_particles;
    std::map<const Particle*, std::vector<std::pair<const Particle*, float>>> particleNeighborsTable;
    std::map<const Particle*, double> nearestNeighborDistanceMap;

    //initializes the particles that will be used
	void initParticles();
    void findNeighbors();
    void updateParticleStates();

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
        
    void processTraj(const common_msgs::Swarm_traj& swarmtraj);
    void calaDynamicBound();
    bool isNearVirtualParticle(double x, double y, double z);
    void generateVirtualParticles(const double l, const int particlesPerSide, const common_msgs::Position& apex);
    void parallelDensityAndPressures();
    void parallelForces();
    void parallelViscosity();
    void parallelUpdateParticleTraj();
    void parallelUpdateParticlePositions(const float deltaTime);
    void pubroscmd();

    inline std::string stateToString(ParticleState state) {
        switch (state) {
            case NULL_STATE: return "SPH";
            case TRAJ:      return "TRAJ";
            case NEED_TRAJ: return "NEED";
            case ATTRACT:   return "ATTRACT";
            case REPEL:     return "REPEL";
            default:        return "UNKNOWN"; // 处理未知状态
        }
    };
    inline double clamp(double value, double max_value) 
    {
        if (value > max_value) {
            return max_value;
        } else if (value < -max_value) {
            return -max_value;
        }
        return value;
    }

};

#endif