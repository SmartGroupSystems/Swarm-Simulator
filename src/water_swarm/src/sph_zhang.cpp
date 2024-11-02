#include "sph_zhang.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_particles");
    ros::NodeHandle nh("~");

    nh.param("sph/particleCount", particleCount, -1);
    nh.param("sph/particleInterval", particleInterval, 0.1);
    nh.param("sph/particleVisScale", particleVisScale, 0.1);
    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1.0f);
    nh.param("sph/h", h, 0.15f);
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/updateInterval", updateInterval, 0.01);
    nh.param("sph/threshold_dist", threshold_dist, 0.1);
    nh.param("sph/k_den", k_den, -1.0);
    nh.param("sph/k_rep", k_rep, -1.0);
    nh.param("sph/k_fri", k_fri, -1.0);
    nh.param("sph/v_max", v_max, -1.0);
    nh.param("sph/a_max", a_max, -1.0);
    nh.param("sph/r_1",   r_1, -1.0);
    nh.param("sph/r_2",   r_2, -1.0);
    nh.param("sph/state_enabled", state_enabled, false);  // 默认值为 false
    nh.param("sph/vis_role", vis_role, false);  // 默认值为 false
    nh.param("sph/init_bias", init_bias, -1.0);  //
    timer                 = nh.createTimer(ros::Duration(updateInterval),   timerCallback);
    particles_publisher   = nh.advertise<visualization_msgs::MarkerArray>("particles_vis", 10);
    virtual_particles_publisher = nh.advertise<visualization_msgs::MarkerArray>("virtual_particles_vis", 10);
    swarm_pub             = nh.advertise<common_msgs::Swarm_particles>("/swarm_particles", 10);
    swarm_traj_sub        = nh.subscribe("/swarm_traj", 1000, swarmTrajCallback);
    target_sub            = nh.subscribe("/particle_target", 10,targetCallback);
    force_sub             = nh.subscribe("/particle_force", 10,forceCallback);
    odom_publishers.resize(particleCount);
    for (int i = 0; i < particleCount; ++i) {
        std::stringstream ss;
        ss << "/particle" << i << "/odom";
        odom_publishers[i] = nh.advertise<nav_msgs::Odometry>(ss.str(), 10);
    }

    //start sph
    SPHSettings sphSettings(mass, restDensity, h, g);
    sph_planner = new SPHSystem(15, sphSettings, false);

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
}

void swarmTrajCallback(const common_msgs::Swarm_traj::ConstPtr& msg)
{
    common_msgs::Swarm_traj swarmTrajBuffer = *msg;
    sph_planner->processTraj(swarmTrajBuffer);
    ROS_INFO("\033[1;32m RECEIVE SWARM TRAJ. \033[0m");
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

void targetCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    for (const auto& particle : msg->particles) {
        sph_planner-> targetMap[particle.index] = particle.position;

        ROS_INFO("Stored particle with index %d: [%f, %f, %f]", 
                particle.index, particle.position.x, particle.position.y, particle.position.z);
    }
    receive_target = true;
}

void forceCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    sph_planner->forceMap.clear();
    for (const auto& particle : msg->particles) {
        sph_planner-> forceMap[particle.index] = particle.force;
    }
    // ROS_INFO("Received forces for %lu particles.", msg->particles.size());
}

void SPHSystem::processTraj(const common_msgs::Swarm_traj& swarmtraj)
{
    // swarmTrajBuffer_.clear();

    for (const auto& traj : swarmtraj.traj)
    {
        // 判断轨迹是否为空（基于 position 列表是否为空）
        if (!traj.position.empty()) {
            swarmTrajBuffer_[traj.traj_id] = traj;
        }
    }

    ROS_INFO("Swarm trajectory processed, %lu trajectories stored.", swarmTrajBuffer_.size());
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
            p.position.y = row * particleInterval-init_bias;
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
            p.index= i;

            // 设置粒子名称，例如"Particle 1", "Particle 2"等
            std::stringstream ss;
            ss << "Particle " << i + 1;
            p.name.data = ss.str();

            // 将粒子加入列表
            particles.push_back(p);
        }

    common_msgs::Position apex;
    apex.x = -0.1; apex.y = -0.1; apex.z = 0;
    generateVirtualParticles(maxRange,static_cast<int>(std::sqrt(particleCount))*3,apex);

    // std::cout << "\033[32m粒子初始化成功！总计初始化 " << particles.size()
    //               << " 个粒子，在 [" << 0 << ", " << 0 << "] 到 [" << maxRange << ", " << maxRange 
    //               << "] 的正方形区域内。\033[0m" << std::endl;

    // std::cout << "\033[31m虚拟粒子初始化成功！总计初始化 " << virtual_particles.size()
    //           << " 个粒子。\033[0m" << std::endl;
    
}

void SPHSystem::findNeighbors() {

    // 清空先前的邻居表
    particleNeighborsTable.clear();
    nearestNeighborDistanceMap.clear();

    for (auto& particle : particles) {
        std::vector<std::pair<const Particle*, float>> neighbors;
        double minDistance = std::numeric_limits<double>::max();

        for (auto& other : particles) {
            if (&particle == &other) continue;

            float dx = particle.position.x - other.position.x;
            float dy = particle.position.y - other.position.y;
            float dz = particle.position.z - other.position.z;
            float dist2 = dx * dx + dy * dy + dz * dz;
            float h2 = settings.h * settings.h;

            if (dist2 < 4 * h2) {
                neighbors.push_back(std::make_pair(&other, dist2));
                minDistance = std::min(minDistance, static_cast<double>(dist2));
            }
        }

        particleNeighborsTable[&particle] = neighbors;

        if (!neighbors.empty()) {
            nearestNeighborDistanceMap[&particle] = std::sqrt(minDistance);  // 记录实际距离（平方根）
        } else {
            nearestNeighborDistanceMap[&particle] = -1.0;  // 没有邻居
        }
    }

}

// void SPHSystem::updateParticleStates()
// {
//     // 遍历每个粒子
//     for (auto& particle : particles) {
//         int particleIndex = particle.index;

//         const auto& targetPosition = targetMap[particleIndex];
    
//         // 计算粒子当前位置与目标点的距离
//         double distanceToTarget = std::sqrt(
//             std::pow(particle.position.x - targetPosition.x, 2) +
//             std::pow(particle.position.y - targetPosition.y, 2) +
//             std::pow(particle.position.z - targetPosition.z, 2)
//         );

//         if (distanceToTarget < 1.0) {
//             particle.state = NEAR_TARGET;
//             continue;  //
//         }

//         // 检查粒子是否有轨迹（从swarmTrajBuffer_中查询）
//         if (swarmTrajBuffer_.find(particleIndex) != swarmTrajBuffer_.end() &&
//             !swarmTrajBuffer_[particleIndex].position.empty()) {
//             // 粒子进入TRAJ状态
//             particle.state = TRAJ;
//         } else {
//             // 如果轨迹为空，保持或进入NULL_STATE
//             particle.state = NULL_STATE;
//             continue;  // 如果是NULL_STATE，跳过剩余判断
//         }

//         // 获取最近邻居的距离
//         double nearestDistance = nearestNeighborDistanceMap[&particle];
//         float h = settings.h;  

//         // 检查粒子的状态变化逻辑
//         if (nearestDistance < r_1 * h && nearestDistance >= 0.0) {
//             // 距离小于 r1 * h，进入REPEL状态
//             particle.state = REPEL;
//             // 清空该粒子的轨迹向量
//             swarmTrajBuffer_[particleIndex].position.clear();
//             continue;  // 如果进入REPEL状态，跳过剩余判断
//         } 
//         else if (nearestDistance > r_2 * h) {
//             // 距离大于 r_2 * h，进入ATTRACT状态
//             particle.state = ATTRACT;
//             // 清空该粒子的轨迹向量
//             swarmTrajBuffer_[particleIndex].position.clear();
//             continue;  // 如果进入ATTRACT状态，跳过剩余判断
//         } 
//         else {
//             // 距离在 r1 * h 和 r_2 * h 之间，保持TRAJ状态
//             particle.state = TRAJ;
//             continue;  // 保持TRAJ状态后，跳过剩余判断
//         }

//         // 当从ATTRACT或REPEL状态脱离时，重新进入TRAJ状态
//         if ((particle.state == ATTRACT || particle.state == REPEL) &&
//             (nearestDistance >= r_1 * h && nearestDistance <= r_2 * h)) {
//             particle.state = NEED_TRAJ;
//             continue;  // 进入NEED_TRAJ状态后，跳过剩余判断
//         }

//     }

//     // // 打印所有粒子的状态
//     // std::cout << "\033[1;33m粒子状态: "; 
//     // for (const auto& particle : particles) {
//     //     std::cout << stateToString(particle.state) << "  ";  
//     // }
//     // std::cout << "\033[0m" << std::endl;  
// }

// void SPHSystem::updateParticleRole()
// {
//     for (auto& particle : particles)
//     {
//         // 获取目标点
//         auto target_it = targetMap.find(particle.index);
        
//         // 如果没有收到目标点，将粒子的角色设为 FREE
//         if (!receive_target) 
//         {
//             particle.role = FREE;
//             continue;
//         }

//         // 如果收到目标点，获取目标点坐标
//         const auto& target_position = target_it->second;

//         // 计算前进方向
//         Eigen::Vector3d forward_dir;
//         forward_dir.x() = target_position.x - particle.position.x;
//         forward_dir.y() = target_position.y - particle.position.y;
//         forward_dir.z() = target_position.z - particle.position.z;
//         forward_dir.normalize();

//         // 检查邻居是否有位于前进方向上的粒子
//         bool found_leader = false;
//         if (particleNeighborsTable.count(&particle) > 0)
//         {
//             for (const auto& neighbor_pair : particleNeighborsTable[&particle])
//             {
//                 const Particle* neighbor = neighbor_pair.first;

//                 // 计算邻居相对于当前粒子的方向
//                 Eigen::Vector3d neighbor_dir(
//                     neighbor->position.x - particle.position.x,
//                     neighbor->position.y - particle.position.y,
//                     neighbor->position.z - particle.position.z
//                 );
//                 neighbor_dir.normalize();

//                 // 判断邻居是否在前进方向上
//                 if (forward_dir.dot(neighbor_dir) > 0.5) // 
//                 {
//                     found_leader = true;
//                     break;
//                 }
//             }
//         }

//         // 根据是否找到前进方向上的邻居设置角色
//         particle.role = found_leader ? FOLLOWER : LEADER;
//     }
// }

void SPHSystem::updateParticleRole()
{
    for (auto& particle : particles)
    {
        // 获取目标点
        // auto target_it = targetMap.find(particle.index);
        
        // 如果没有收到目标点，将粒子的角色设为 FREE
        if (!receive_target||particle.state == NEAR_TARGET) 
        {
            particle.role = FREE;
            continue;
        }

        // 记录四个象限的邻居情况
        bool quadrant1 = false;  // x > 0, y > 0, z > 0
        bool quadrant2 = false;  // x < 0, y > 0, z > 0
        bool quadrant3 = false;  // x < 0, y < 0, z > 0
        bool quadrant4 = false;  // x > 0, y < 0, z > 0

        // 检查邻居是否分布在四个象限
        if (particleNeighborsTable.count(&particle) > 0)
        {
            for (const auto& neighbor_pair : particleNeighborsTable[&particle])
            {
                const Particle* neighbor = neighbor_pair.first;

                // 计算邻居相对于当前粒子的方向
                Eigen::Vector3d relative_dir(
                    neighbor->position.x - particle.position.x,
                    neighbor->position.y - particle.position.y,
                    neighbor->position.z - particle.position.z
                );

                // 判断邻居位于哪个象限
                if (relative_dir.x() > 0 && relative_dir.y() > 0 )
                    quadrant1 = true;
                else if (relative_dir.x() < 0 && relative_dir.y() > 0 )
                    quadrant2 = true;
                else if (relative_dir.x() < 0 && relative_dir.y() < 0 )
                    quadrant3 = true;
                else if (relative_dir.x() > 0 && relative_dir.y() < 0 )
                    quadrant4 = true;

                // 如果四个象限都已有邻居，提前退出
                if (quadrant1 && quadrant2 && quadrant3 && quadrant4)
                    break;
            }
        }

        // 根据象限分布设置角色
        particle.role = (quadrant1 && quadrant2 && quadrant3 && quadrant4) ? FOLLOWER : LEADER;
    }
}


void SPHSystem::updateParticleStates()
{
    // 遍历每个粒子
    for (auto& particle : particles) {
        int particleIndex = particle.index;
        const auto& targetPosition = targetMap[particleIndex];
    
        // 计算粒子当前位置与目标点的距离
        double distanceToTarget = std::sqrt(
            std::pow(particle.position.x - targetPosition.x, 2) +
            std::pow(particle.position.y - targetPosition.y, 2) +
            std::pow(particle.position.z - targetPosition.z, 2)
        );

        if (distanceToTarget < 1.0) {
            particle.state = NEAR_TARGET;
            continue;  //
        }

        if( particle.role == FOLLOWER ||particle.role == FREE)
        {
            particle.state = NULL_STATE;
            continue;
        }

        // 检查粒子是否有轨迹（从swarmTrajBuffer_中查询）
        if (swarmTrajBuffer_.find(particleIndex) != swarmTrajBuffer_.end() &&
            !swarmTrajBuffer_[particleIndex].position.empty()) {
            // 粒子进入TRAJ状态
            particle.state = TRAJ;
        } else {
            // 如果轨迹为空，保持或进入NULL_STATE
            particle.state = NULL_STATE;
            continue;  // 如果是NULL_STATE，跳过剩余判断
        }

        // 获取最近邻居的距离
        double nearestDistance = nearestNeighborDistanceMap[&particle];
        // ROS_INFO("Particle Index: %d, Nearest Distance: %.4f", particle.index, nearestDistance);
        float h = settings.h;  
        // 检查粒子的状态变化逻辑
        if (nearestDistance < r_1 * h && nearestDistance >= 0.0) {
            // 距离小于 r1 * h，进入REPEL状态
            particle.state = REPEL;
            // 清空该粒子的轨迹向量
            swarmTrajBuffer_[particleIndex].position.clear();
            continue;  // 如果进入REPEL状态，跳过剩余判断
        } 
        else if (nearestDistance > r_2 * h) {
            // 距离大于 r_2 * h，进入ATTRACT状态
            particle.state = ATTRACT;
            // 清空该粒子的轨迹向量
            swarmTrajBuffer_[particleIndex].position.clear();
            continue;  // 如果进入ATTRACT状态，跳过剩余判断
        } 
        else {
            // 距离在 r1 * h 和 r_2 * h 之间，保持TRAJ状态
            particle.state = TRAJ;
            continue;  // 保持TRAJ状态后，跳过剩余判断
        }

        // 当从ATTRACT或REPEL状态脱离时，重新进入TRAJ状态
        if ((particle.state == ATTRACT || particle.state == REPEL) &&
            (nearestDistance >= r_1 * h && nearestDistance <= r_2 * h)) {
            particle.state = NEED_TRAJ;
            continue;  // 进入NEED_TRAJ状态后，跳过剩余判断
        }

    }

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

    updateParticleRole();

    updateParticleStates();

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
        float neighborGrad = 0.0;

        particle.u_den.x = 0.0;
        particle.u_den.y = 0.0;
        particle.u_den.z = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        for (auto& neighbor_pair : neighbors) {

            float dist2 = neighbor_pair.second;
            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = 1-(3/2)*k*k + (3/4)*k*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (1/4)*(2-k)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            pDensity += settings.poly * q;
        }

        particle.density = pDensity + settings.selfDens;
        // double k_den_coefficient = (forceMap.count(particle.index) > 0 && forceMap[particle.index].k_den != 0) ? forceMap[particle.index].k_den : 1.0;
        // if (k_den_coefficient > 1.0)
        // {
        //     ROS_INFO("k_den_coefficient for particle %d: %f", particle.index, k_den_coefficient);
        // }
        
        double p = -k_den  * (1/particle.density) * (std::pow(particle.density/settings.restDensity,7) -1 );
        
        for (auto& neighbor_pair : neighbors)
        {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;
            float dist = std::sqrt(dist2);

            // 计算方向向量
            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;
       
            // 归一化方向向量
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = -3*k + (9/4)*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (-3/4)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            neighborGrad = settings.poly * h * q;

            particle.u_den.x += p * neighborGrad * dir_x;
            particle.u_den.y += p * neighborGrad * dir_y;
            particle.u_den.z += p * neighborGrad * dir_z;

        }
    }
}

void SPHSystem::parallelForces()
{
    for (auto& particle : particles) {

        particle.u_rep.x = 0.0;
        particle.u_rep.y = 0.0;
        particle.u_rep.z = 0.0;

        particle.u_fri.x = 0.0; 
        particle.u_fri.y = 0.0; 
        particle.u_fri.z = 0.0; 

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

            particle.u_rep.x += k_rep * 1/dist2 * dist * dir_x;
            particle.u_rep.y += k_rep * 1/dist2 * dist * dir_y;
            particle.u_rep.z += k_rep * 1/dist2 * dist * dir_z;

        }
    
        particle.u_fri.x = -k_fri * particle.velocity.x; 
        particle.u_fri.y = -k_fri * particle.velocity.y; 
        particle.u_fri.z = -k_fri * particle.velocity.z; 
    }   
}


void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particles.size(); i++) {
        Particle *p = &particles[i];

        // 计算加速度 u_i = u_i^{den} + u_i^{rep} + u_i^{fri} + traj.a
        common_msgs::Acceleration acceleration;
        acceleration.x = 0.0f;
        acceleration.y = 0.0f;
        acceleration.z = 0.0f;

        auto itt = swarmTrajBuffer_.end(); // 初始化为无效的迭代器
        bool traj_found = false;

        // 查找粒子的轨迹
        if (p->state == TRAJ || p->state == NEED_TRAJ) {
            itt = swarmTrajBuffer_.find(p->index);
            if (itt != swarmTrajBuffer_.end()) {
                traj_found = true;
            }
        }
        
        // 根据粒子的状态来计算加速度
        switch (p->state) {
            case NULL_STATE:
                // NULL_STATE 加速度计算
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x + forceMap[p->index].x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y + forceMap[p->index].y;
                acceleration.z = p->u_den.z + p->u_rep.z + p->u_fri.z + forceMap[p->index].z;
                break;

            case NEAR_TARGET:
                // NULL_STATE 加速度计算
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x + forceMap[p->index].x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y + forceMap[p->index].y;
                acceleration.z = p->u_den.z + p->u_rep.z + p->u_fri.z + forceMap[p->index].z;
                break;

            case ATTRACT:
                // ATTRACT 状态 加速度计算
                acceleration.x = p->u_den.x + p->u_fri.x + forceMap[p->index].x;
                acceleration.y = p->u_den.y + p->u_fri.y + forceMap[p->index].y;
                acceleration.z = p->u_den.z + p->u_fri.z + forceMap[p->index].z;
                break;

            case REPEL:
                // REPEL 状态 加速度计算
                acceleration.x = p->u_den.x + p->u_rep.x + forceMap[p->index].x;
                acceleration.y = p->u_den.y + p->u_rep.y + forceMap[p->index].y;
                acceleration.z = p->u_den.z + p->u_rep.z + forceMap[p->index].z;
                break;

            case TRAJ:
            case NEED_TRAJ:
                // TRAJ 或 NEED_TRAJ 状态 加速度计算
                if (traj_found) {
                    const auto& traj_acceleration = itt->second.acceleration;
                    if (!traj_acceleration.empty()) {
                        acceleration.x += traj_acceleration[0].x;
                        acceleration.y += traj_acceleration[0].y;
                        acceleration.z += traj_acceleration[0].z;
                    }
                }
                break;
            
            default:
                std::cerr << "Error: Unknown particle state!" << std::endl;
                break;
        }

        //Updare traj
        parallelUpdateParticleTraj();
        //  ROS_INFO("Acceleration - x: %f, y: %f, z: %f", acceleration.x, acceleration.y, acceleration.z);
        //加速度限制
        acceleration.x = clamp(acceleration.x, a_max);
        acceleration.y = clamp(acceleration.y, a_max);
        acceleration.z = clamp(acceleration.z, a_max);  

        //update acc
        p->acceleration.x = acceleration.x;
        p->acceleration.y = acceleration.y;
        p->acceleration.z = acceleration.z;

        // 更新速度
        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;

        //速度限制
        p->velocity.x = clamp(p->velocity.x, v_max);
        p->velocity.y = clamp(p->velocity.y, v_max);
        p->velocity.z = clamp(p->velocity.z, v_max);


        // 更新位置
        p->position.x += p->velocity.x * deltaTime;
        p->position.y += p->velocity.y * deltaTime;
        p->position.z += p->velocity.z * deltaTime;

    }
     
}


// void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
// {
//     for (size_t i = 0; i < particles.size(); i++) {
//         Particle *p = &particles[i];

//         // 计算加速度 u_i = u_i^{den} + u_i^{rep} + u_i^{fri} + traj.a
//         common_msgs::Acceleration acceleration;
//         acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x;
//         acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y;
//         acceleration.z = p->u_den.z + p->u_rep.z + p->u_fri.z;

//         std::cout << "Acceleration: (" 
//                 << acceleration.x << ", " 
//                 << acceleration.y << ", " 
//                 << acceleration.z << ")"
//                 << " | u_den: (" 
//                 << p->u_den.x << ", " 
//                 << p->u_den.y << ", " 
//                 << p->u_den.z << ")"
//                 << " | u_rep: (" 
//                 << p->u_rep.x << ", " 
//                 << p->u_rep.y << ", " 
//                 << p->u_rep.z << ")"
//                 << " | u_fri: (" 
//                 << p->u_fri.x << ", " 
//                 << p->u_fri.y << ", " 
//                 << p->u_fri.z << ")" 
//                 << std::endl;

//         // 获取对应的粒子轨迹的加速度
//         auto itt = swarmTrajBuffer_.find(p->index); // 根据粒子索引查找轨迹
//         if (itt != swarmTrajBuffer_.end()) {
//             // 获取加速度的第一个值并添加到当前加速度
//             const auto& traj_acceleration = itt->second.acceleration;
//             if (!traj_acceleration.empty()) {
//                 acceleration.x += traj_acceleration[0].x;
//                 acceleration.y += traj_acceleration[0].y;
//                 acceleration.z += traj_acceleration[0].z;
//             }
//                 // std::cout << "Particle index: " << p->index
//                 //   << " | Traj Acceleration: ("
//                 //   << traj_acceleration[0].x << ", "
//                 //   << traj_acceleration[0].y << ", "
//                 //   << traj_acceleration[0].z << ")"
//                 //   << " | Updated Acceleration: ("
//                 //   << acceleration.x << ", "
//                 //   << acceleration.y << ", "
//                 //   << acceleration.z << ")" << std::endl;
// std::cout << "\033[32m"  // 32m是绿色
//           << "Trajectory Acceleration: ("
//           << traj_acceleration[0].x << ", "
//           << traj_acceleration[0].y << ", "
//           << traj_acceleration[0].z << "), "
//           << "Trajectory Velocity: ("
//           << itt->second.velocity[0].x << ", "
//           << itt->second.velocity[0].y << ", "
//           << itt->second.velocity[0].z << ")"
//           << "\033[0m"  // 重置颜色
//           << std::endl;

//         }
//         //Updare traj
//         parallelUpdateParticleTraj();

//         //加速度限制
//         acceleration.x = clamp(acceleration.x, a_max);
//         acceleration.y = clamp(acceleration.y, a_max);
//         acceleration.z = clamp(acceleration.z, a_max);  

//         // 更新速度
//         p->velocity.x += acceleration.x * deltaTime;
//         p->velocity.y += acceleration.y * deltaTime;
//         p->velocity.z += acceleration.z * deltaTime;

//     // std::cout << "deltaTime: " << deltaTime << std::endl;
//         // // 获取对应的粒子轨迹的速度
//         // auto it = swarmTrajBuffer_.find(p->index); // 根据粒子索引查找轨迹
//         // if (it != swarmTrajBuffer_.end()) {
//         //     // 获取加速度的第一个值并添加到当前加速度
//         //     const auto& traj_velocity = it->second.velocity;
//         //     if (!traj_velocity.empty()) {
//         //         p->velocity.x += traj_velocity[0].x;
//         //         p->velocity.y += traj_velocity[0].y;
//         //         p->velocity.z += traj_velocity[0].z;
//         //     }
//         // }

//         //速度限制
//         p->velocity.x = clamp(p->velocity.x, v_max);
//         p->velocity.y = clamp(p->velocity.y, v_max);
//         p->velocity.z = clamp(p->velocity.z, v_max);

//         // std::cout << "Particle Velocity: ("
//         //   << p->velocity.x << ", "
//         //   << p->velocity.y << ", "
//         //   << p->velocity.z << ")" << std::endl;

// std::cout << "Particle Acceleration: ("
//           << acceleration.x << ", "
//           << acceleration.y << ", "
//           << acceleration.z << ") | "
//           << "Particle Velocity: ("
//           << p->velocity.x << ", "
//           << p->velocity.y << ", "
//           << p->velocity.z << ")" 
//           << std::endl;

//         // 更新位置
//         p->position.x += p->velocity.x * deltaTime;
//         p->position.y += p->velocity.y * deltaTime;
//         p->position.z += p->velocity.z * deltaTime;

//         // // 获取对应的粒子轨迹的位置
//         // auto ittt = swarmTrajBuffer_.find(p->index); // 根据粒子索引查找轨迹
//         // if (ittt != swarmTrajBuffer_.end()) {
//         //     // 获取加速度的第一个值并添加到当前加速度
//         //     const auto& traj_position = ittt->second.position;
//         //     if (!traj_position.empty()) {
//         //         p->position.x += traj_position[0].x;
//         //         p->position.y += traj_position[0].y;
//         //         p->position.z += traj_position[0].z;
//         //     }
//         // }
//             // std::cout << "Particle index: " << p->index 
//             //   << " | Acceleration: x=" << acceleration.x 
//             //   << ", y=" << acceleration.y 
//             //   << ", z=" << acceleration.z << std::endl;
//             // std::cout << "Particle index: " << p->index 
//             //   << " | Velocity: x=" << p->velocity.x 
//             //   << ", y=" << p->velocity.y 
//             //   << ", z=" << p->velocity.z << std::endl;
//             // std::cout << "Particle index: " << p->index 
//             //   << " | Position: x=" << p->position.x 
//             //   << ", y=" << p->position.y 
//             //   << ", z=" << p->position.z << std::endl;
//     }
     
// }

void SPHSystem::pubroscmd() 
{
    visualization_msgs::MarkerArray particles_markers;
    visualization_msgs::MarkerArray virtual_particles_markers;
    visualization_msgs::MarkerArray state_text_markers; 
    common_msgs::Swarm_particles swarm_msg;  

    // 为每个实际粒子设置 Marker 并填充 Swarm_particles 消息
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
        marker.id = particle.index;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = particleVisScale;
        marker.scale.y = particleVisScale;
        marker.scale.z = particleVisScale;
        if (particle.role == LEADER)
        {
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        particles_markers.markers.push_back(marker);

        if (state_enabled)
        {
            // 创建状态文本标记
            visualization_msgs::Marker state_marker;
            state_marker.header.frame_id = "world";
            state_marker.header.stamp = ros::Time::now();
            state_marker.ns = "state_texts";
            state_marker.action = visualization_msgs::Marker::ADD;
            state_marker.pose.position.x = particle.position.x;
            state_marker.pose.position.y = particle.position.y; // 让文本稍微高于粒子
            state_marker.pose.position.z = particle.position.z + particleVisScale + 0.4;
            state_marker.pose.orientation.w = 1.0;
            state_marker.id = particle.index + particles.size(); // 确保 ID 唯一
            state_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; // 使用文本视图标记
            state_marker.scale.z = 0.5; // 字体大小，与你的粒子大小适配
            state_marker.color.a = 1.0;
            state_marker.color.r = 0.0; // 黑色
            state_marker.color.g = 0.0;
            state_marker.color.b = 0.0;

            // 设置状态文本   
            if (vis_role)
            {
                state_marker.text = roleToString(particle.role);
            }
            else
            {
                state_marker.text = stateToString(particle.state);
            }
            
            state_text_markers.markers.push_back(state_marker);
        }
        
        // 同时将粒子信息添加到 Swarm_particles 消息中
        common_msgs::Particle swarm_particle;
        swarm_particle.position = particle.position;
        swarm_particle.velocity = particle.velocity;
        swarm_particle.acceleration = particle.acceleration;
        swarm_particle.force = particle.force;
        swarm_particle.density = particle.density;
        swarm_particle.pressure = particle.pressure;
        swarm_particle.index = particle.index;
        swarm_particle.state = particle.state;
        swarm_particle.role  = particle.role;
        swarm_msg.particles.push_back(swarm_particle);
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
        marker.scale.x = particleVisScale;
        marker.scale.y = particleVisScale;
        marker.scale.z = particleVisScale;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        virtual_particles_markers.markers.push_back(marker);
    }

    //pub particles odom
    for (const auto& particle : particles) {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = "particle" + std::to_string(particle.index);

        odom.pose.pose.position.x = particle.position.x;
        odom.pose.pose.position.y = particle.position.y;
        odom.pose.pose.position.z = particle.position.z;
        odom.pose.pose.orientation.w = 1.0;  // Assuming no orientation information

        odom.twist.twist.linear.x = particle.velocity.x;
        odom.twist.twist.linear.y = particle.velocity.y;
        odom.twist.twist.linear.z = particle.velocity.z;

        // 使用与 particle.index 对应的 Publisher 发布消息
        if (particle.index < odom_publishers.size()) {
            odom_publishers[particle.index].publish(odom);
        }
    }

    // 发布所有粒子
    particles_publisher.publish(particles_markers);
    virtual_particles_publisher.publish(virtual_particles_markers);
    particles_publisher.publish(state_text_markers);

    // 发布 Swarm_particles 消息
    swarm_msg.header.stamp = ros::Time::now();  // 设置时间戳
    swarm_msg.header.frame_id = "world";  // 设置 frame_id
    swarm_pub.publish(swarm_msg);  // 发布消息
}

void SPHSystem::calaDynamicBound()
{

}

void SPHSystem::parallelUpdateParticleTraj() {
    // 遍历所有的粒子轨迹
    for (auto& entry : swarmTrajBuffer_) {
        auto& traj = entry.second;

        // 如果轨迹为空，返回
        if (traj.position.empty() || traj.velocity.empty() ||
            traj.acceleration.empty() || traj.jerk.empty()) {
            continue;
        }

        // 弹出每条轨迹的第一个位置，速度，加速度，jerk量
        traj.position.erase(traj.position.begin());
        traj.velocity.erase(traj.velocity.begin());
        traj.acceleration.erase(traj.acceleration.begin());
        traj.jerk.erase(traj.jerk.begin());
    }
}



// 生成边长为 l 的正方形边界上的虚拟粒子
void SPHSystem::generateVirtualParticles(const double l, const int particlesPerSide, const common_msgs::Position& apex) {

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
