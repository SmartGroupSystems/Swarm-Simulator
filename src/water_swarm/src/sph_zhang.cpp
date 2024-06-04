#include "sph_zhang.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_particles");
    ros::NodeHandle nh("~");

    nh.param("sph/particleCount", particleCount, -1);
    nh.param("sph/particleInterval", particleInterval, 0.1);
    nh.param("sph/particleVisScale", particleVisScale, 0.1);
    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1000.0f);
    nh.param("sph/gasConstant", gasConstant, 1.0f);
    nh.param("sph/viscosity", viscosity, 1.04f);
    nh.param("sph/h", h, 0.15f);//
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/tension", tension, 0.2f);
    nh.param("sph/updateInterval", updateInterval, 0.01);
    nh.param("sph/threshold_dist", threshold_dist, 0.1);

    timer                 = nh.createTimer(ros::Duration(updateInterval),   timerCallback);
    nav_goal_sub          = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
    particles_publisher   = nh.advertise<visualization_msgs::MarkerArray>("particles_vis", 10);
    virtual_particles_publisher = nh.advertise<visualization_msgs::MarkerArray>("virtual_particles_vis", 10);


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
    
    current_time = ros::Time::now();

    if (!sph_planner->started) 
    {
        // 检查自上次打印后是否已经过去了1秒
        if ((current_time - last_print_time).toSec() >= 1.0) {
            ROS_INFO("Do not receive the start command, return.");
            last_print_time = current_time;  // 更新上次打印时间
        }
        return;
    }
    else
    {   
        // 计算 deltaTime，如果是第一次进入，则将其设置为0，这个地方需要在这个时间间隔里执行多次update循环
        float deltaTime = last_time.isZero() ? 0.0 : (current_time - last_time).toSec();

        // 更新last_time为当前时间
        last_time = current_time;

        sph_planner->update(updateInterval);
    }
}

void SPHSystem::initParticles()
{
    sph_particleCount = particleCount;

    int sideLength = static_cast<int>(std::sqrt(particleCount));
        if (sideLength * sideLength < particleCount) {
            ++sideLength;
        }
    double maxRange = sideLength * particleInterval;
    
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

    water_swarm::Position apex;
    apex.x = -0.1; apex.y = -0.1; apex.z = 0;
    generateVirtualParticles(maxRange,static_cast<int>(std::sqrt(particleCount))*3,apex);

    std::cout << "\033[32m粒子初始化成功！总计初始化 " << particles.size()
                  << " 个粒子，在 [" << 0 << ", " << 0 << "] 到 [" << maxRange << ", " << maxRange 
                  << "] 的正方形区域内。\033[0m" << std::endl;

    std::cout << "\033[31m虚拟粒子初始化成功！总计初始化 " << virtual_particles.size()
              << " 个粒子。\033[0m" << std::endl;
    
}

void SPHSystem::findNeighbors() {

    // 清空先前的邻居表
    particleNeighborsTable.clear();

    for (auto& particle : particles) {
        std::vector<std::pair<const Particle*, float>> neighbors;

        for (auto& other : particles) {
            if (&particle == &other) continue;

            float dx = particle.position.x - other.position.x;
            float dy = particle.position.y - other.position.y;
            float dz = particle.position.z - other.position.z;
            float dist2 = dx * dx + dy * dy + dz * dz;
            float h2 = settings.h * settings.h;

            if (dist2 < h2) {
                neighbors.push_back(std::make_pair(&other, dist2));
            }
        }

        particleNeighborsTable[&particle] = neighbors;
    }

    // // 输出邻居信息作为测试
    // for (const auto& pair : particleNeighborsTable) {
    //     const Particle* particle = pair.first;
    //     const std::vector<std::pair<const Particle*, float>>& neighbors = pair.second;

    //     std::cout << particle->name.data << " 的邻居数量: " << neighbors.size() << std::endl;

    //     for (const auto& neighbor_pair : neighbors) {
    //         const Particle* neighbor = neighbor_pair.first;
    //         float dist2 = neighbor_pair.second;
    //         std::cout << " - 邻居: " << neighbor->name.data
    //                   << ", 距离平方: " << std::fixed << std::setprecision(2) << dist2 << std::endl;
    //     }
    // }

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
    findNeighbors();

    // Calculate densities and pressures
    parallelDensityAndPressures();

    // Calculate forces
    parallelForces();

    // Update particle positions and neighbors
    parallelUpdateParticlePositions(deltaTime);

    //rospub control commands
    pubroscmd();
}

void SPHSystem::parallelDensityAndPressures()
{   
    for (auto& particle : particles) {

        float pDensity = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        for (auto& neighbor_pair : neighbors) {
            // const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;

            pDensity += settings.massPoly6Product * std::pow(settings.h2 - dist2, 3);
        }

        particle.density = pDensity + settings.selfDens;
        particle.pressure = settings.gasConstant * (particle.density - settings.restDensity);
    }
}

void SPHSystem::parallelForces()
{
    for (auto& particle : particles) {

        // 重置粒子的力
        particle.force.x = 0.0;
        particle.force.y = 0.0;
        particle.force.z = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        
        for (auto& neighbor_pair : neighbors) {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;

            // 计算邻居与当前粒子之间的实际距离
            float dist = std::sqrt(dist2);

            // 计算方向向量
            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;

            // 归一化方向向量
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            // 计算压力力
            float pressure = (particle.pressure + neighbor->pressure) / (2.0 * neighbor->density);

            water_swarm::Force pressureForce;
            pressureForce.x = -dir_x * settings.mass * pressure * settings.spikyGrad;
            pressureForce.y = -dir_y * settings.mass * pressure * settings.spikyGrad;
            pressureForce.z = -dir_z * settings.mass * pressure * settings.spikyGrad;
            // pressureForce.x = dir_x * settings.mass * pressure * settings.spikyGrad;
            // pressureForce.y = dir_y * settings.mass * pressure * settings.spikyGrad;
            // pressureForce.z = dir_z * settings.mass * pressure * settings.spikyGrad;


            // 调整压力力的权重
            pressureForce.x *= std::pow(settings.h - dist, 2);
            pressureForce.y *= std::pow(settings.h - dist, 2);
            pressureForce.z *= std::pow(settings.h - dist, 2);

             // 计算粘性力
            float dvx = neighbor->velocity.x - particle.velocity.x;
            float dvy = neighbor->velocity.y - particle.velocity.y;
            float dvz = neighbor->velocity.z - particle.velocity.z;

            water_swarm::Force viscosityForce;
            viscosityForce.x = settings.mass * settings.viscosity * dvx / neighbor->density * settings.spikyLap * (settings.h - dist);
            viscosityForce.y = settings.mass * settings.viscosity * dvy / neighbor->density * settings.spikyLap * (settings.h - dist);
            viscosityForce.z = settings.mass * settings.viscosity * dvz / neighbor->density * settings.spikyLap * (settings.h - dist);

            // 将压力力和粘性力累加到粒子 `pi` 上
            particle.force.x += pressureForce.x + viscosityForce.x;
            particle.force.y += pressureForce.y + viscosityForce.y;
            particle.force.z += pressureForce.z + viscosityForce.z;

        }
       
    }
}

void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particles.size(); i++) {
        Particle *p = &particles[i];

        // 计算加速度
        water_swarm::Acceleration acceleration;
        acceleration.x = p->force.x / p->density;  // 根据牛顿第二定律计算加速度
        acceleration.y = p->force.y / p->density;
        acceleration.z = p->force.z / p->density;

        // 更新速度
        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;

        // 预计算新位置
        double newX = p->position.x + p->velocity.x * deltaTime;
        double newY = p->position.y + p->velocity.y * deltaTime;
        double newZ = p->position.z + p->velocity.z * deltaTime;

        // 检查是否触碰到虚拟粒子构成的墙
        if (isNearVirtualParticle(newX, newY, newZ)) {
            p->velocity.x = -p->velocity.x;
            p->velocity.y = -p->velocity.y;
        }

        // 更新位置
        p->position.x += p->velocity.x * deltaTime;
        p->position.y += p->velocity.y * deltaTime;
        p->position.z += p->velocity.z * deltaTime;
    }
     
}

void SPHSystem::pubroscmd() 
{
    visualization_msgs::MarkerArray particles_markers;
    visualization_msgs::MarkerArray virtual_particles_markers;

    // 为每个实际粒子设置 Marker
    int id = 0;
    for (const auto& particle : particles) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = particle.position.x;
        marker.pose.position.y = particle.position.y;
        marker.pose.position.z = particle.position.z;
        marker.pose.orientation.w = 1.0;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;  // 更改类型为SPHERE
        marker.scale.x = particleVisScale; // 球的直径，根据需要调整大小
        marker.scale.y = particleVisScale;
        marker.scale.z = particleVisScale;
        marker.color.a = 1.0; // 透明度
        marker.color.r = 0.0; // 红色
        marker.color.g = 1.0; // 绿色
        marker.color.b = 0.0; // 蓝色
        particles_markers.markers.push_back(marker);
    }

    // 为每个虚拟粒子设置 Marker
    id = 0;
    for (const auto& particle : virtual_particles) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "virtual_particles";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = particle.position.x;
        marker.pose.position.y = particle.position.y;
        marker.pose.position.z = particle.position.z;
        marker.pose.orientation.w = 1.0;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = particleVisScale; // 根据需要调整大小
        marker.scale.y = particleVisScale;
        marker.scale.z = particleVisScale;
        marker.color.a = 1.0; // 透明度
        marker.color.r = 0.0; // 黑色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        virtual_particles_markers.markers.push_back(marker);
    }

    // 发布所有粒子
    particles_publisher.publish(particles_markers);
    virtual_particles_publisher.publish(virtual_particles_markers);
}

void SPHSystem::calaDynamicBound()
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

bool SPHSystem::isNearVirtualParticle(double x, double y, double z) 
{
    for (const Particle& vp : virtual_particles) {
        double dx = vp.position.x - x;
        double dy = vp.position.y - y;
        double dz = vp.position.z - z;
        if (sqrt(dx * dx + dy * dy + dz * dz) < threshold_dist) {
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
    /*---TEST-------*/ 
    started = true;
    /*--------------*/

    initParticles();
    findNeighbors();
   
}

SPHSystem::SPHSystem()
    : sph_particleCubeWidth(0.0),
      settings(),
      started(false),
      runOnGPU(false){}

SPHSystem::~SPHSystem() {
}
