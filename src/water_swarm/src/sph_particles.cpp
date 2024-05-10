#include "sph_particles.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_particles");
    ros::NodeHandle nh("~");

    nh.param("sph/particleCount", particleCount, -1);
    nh.param("sph/particleInterval", particleInterval, 0.1);
    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1000.0f);
    nh.param("sph/gasConstant", gasConstant, 1.0f);
    nh.param("sph/viscosity", viscosity, 1.04f);
    nh.param("sph/h", h, 0.15f);//
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/tension", tension, 0.2f);

    timer                 = nh.createTimer(ros::Duration(0.05),   timerCallback);
    nav_goal_sub          = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
    particles_publisher   = nh.advertise<visualization_msgs::Marker>("particles_vis", 10);
    virtual_particles_vis = nh.advertise<visualization_msgs::Marker>("virtual_particles_vis", 10);


    //start sph
    SPHSettings sphSettings(mass, restDensity, gasConstant, viscosity, h, g, tension);
    sph_planner = new SPHSystem(15, sphSettings, false);

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
}


void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sph_planner->started = true;
    ROS_INFO("Received 2D Nav Goal at position: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    
}

void timerCallback(const ros::TimerEvent&) {

}

void SPHSystem::initParticles()
{
    sph_particleCount = particleCount;

    int sideLength = static_cast<int>(std::sqrt(particleCount));
        if (sideLength * sideLength < particleCount) {
            ++sideLength;
        }
    float maxRange = sideLength * particleInterval;
    
        // 初始化粒子数组
        for (int i = 0; i < particleCount; ++i) {
            int row = i / sideLength;
            int col = i % sideLength;
            
            // 初始化每个粒子
            Particle p;
            p.position.x = col * particleInterval;
            p.position.y = row * particleInterval;
            p.position.z = 1.0; // 默认z值为1
            p.velocity.x = 0.0;
            p.velocity.y = 0.0;
            p.velocity.z = 0.0;
            p.acceleration.x = 0.0;
            p.acceleration.y = 0.0;
            p.acceleration.z = 0.0;
            p.force.x = 0.0;
            p.force.y = 0.0;
            p.force.z = 0.0;
            p.density = settings.selfDens;
            p.pressure = 0.0;
            p.hash = 0; // 根据需要设置

            // 设置粒子名称，例如"Particle 1", "Particle 2"等
            std::stringstream ss;
            ss << "Particle " << i + 1;
            p.name.data = ss.str();

            // 将粒子加入列表
            particles.push_back(p);
        }

    std::cout << "\033[32m粒子初始化成功！总计初始化 " << particles.size()
                  << " 个粒子，在 [" << 0 << ", " << 0 << "] 到 [" << maxRange << ", " << maxRange 
                  << "] 的正方形区域内。\033[0m" << std::endl;
    
}

void SPHSystem::update(float deltaTime) {
	if (!started) return;
	// // To increase system stability, a fixed deltaTime is set
	// deltaTime = 0.003f;
    runOnGPU = false;
    updateParticles(particles, particleCount, settings, deltaTime,runOnGPU);
}

void SPHSystem::reset() {
    isInitialReceived = false;
	initParticles();
	started = false;
}

void SPHSystem::start() {
	started = true;
}

void SPHSystem::updateParticles(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime, const bool onGPU)
{
    if (onGPU) {
        updateParticlesGPU(
            particles, particleCount, settings, deltaTime);
    }
    else {
        updateParticlesCPU(
            particles, particleCount, settings, deltaTime);
    }
}

/// CPU update particles implementation
void SPHSystem::updateParticlesCPU(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime)
{
    // Calculate densities and pressures
    parallelDensityAndPressures();

    // Calculate forces
    parallelForces();

    // Update particle positions
    parallelUpdateParticlePositions(deltaTime);

    //rospub control commands
    pubroscmd();
}

void SPHSystem::parallelDensityAndPressures()
{   

}

void SPHSystem::parallelForces()
{
   
}

void SPHSystem::calaDynamicBound()
{

}


void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{

  
}

void SPHSystem::pubroscmd()
{   
    
}

// 生成边长为 l 的正方形边界上的虚拟粒子
void SPHSystem::generateVirtualParticles(const double l, const int particlesPerSide, const water_swarm::Position& apex) {

        float step = l / particlesPerSide;
        // 遍历正方形的四条边
        for (int i = 0; i <= particlesPerSide; ++i) {
            double coordX = apex.x + i * step;
        
            // 上边和下边
            for (double yOffset : std::initializer_list<double>{0, l}) {
            Particle particle;
            particle.position.x = coordX;
            particle.position.y = apex.y + yOffset;
            particle.position.z = apex.z + 1.0; 
            particle.velocity.x = 0;
            particle.velocity.y = 0;
            particle.velocity.z = 0;
            particle.acceleration.x = 0;
            particle.acceleration.y = 0;
            particle.acceleration.z = 0;
            particle.force.x = 0;
            particle.force.y = 0;
            particle.force.z = 0;
            particle.density = 1000; // 假设水的密度
            particle.pressure = 0;
            particle.hash = 0;
            particle.name.data = "Virtual Particle";
            virtual_particles.push_back(particle);
        }

            // 左边和右边，避免角点重复
            if (i != 0 && i != particlesPerSide) {
                for (double xOffset : std::initializer_list<double>{0, l}) {
                Particle particle;
                particle.position.x = apex.x + xOffset;
                particle.position.y = apex.y + i * step;
                particle.position.z = apex.z + 1.0;
                particle.velocity.x = 0;
                particle.velocity.y = 0;
                particle.velocity.z = 0;
                particle.acceleration.x = 0;
                particle.acceleration.y = 0;
                particle.acceleration.z = 0;
                particle.force.x = 0;
                particle.force.y = 0;
                particle.force.z = 0;
                particle.density = 1000; // 假设水的密度
                particle.pressure = 0;
                particle.hash = 0;
                particle.name.data = "Virtual Particle";
                virtual_particles.push_back(particle);
            }
        }
    }
}

bool SPHSystem::isNearVirtualParticle(double x, double y, double z) {
    const double threshold = 0.1;  // 定义碰撞阈值，例如粒子半径
    for (const Particle& vp : virtual_particles) {
        double dx = vp.position.x - x;
        double dy = vp.position.y - y;
        double dz = vp.position.z - z;
        if (sqrt(dx * dx + dy * dy + dz * dz) < threshold) {
            return true;
        }
    }
    return false;
}

SPHSystem::SPHSystem(
    size_t particleCubeWidth, const SPHSettings &settings,
    const bool &runOnGPU)
    : sph_particleCubeWidth(particleCubeWidth)
    , settings(settings)
    , runOnGPU(runOnGPU)
{
    initParticles();
}

SPHSystem::SPHSystem()
    : sph_particleCubeWidth(0.0),
      settings(),
      started(false),
      runOnGPU(false){}

SPHSystem::~SPHSystem() {
}
