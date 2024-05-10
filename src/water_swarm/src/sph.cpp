#include "sph.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_planner");
    ros::NodeHandle nh("~");

// float new_h = 5.0f; // 新的核函数半径
// float volume_per_particle = (4.0f/3.0f) * PI * pow(new_h/2, 3);
// float new_mass = restDensity * volume_per_particle; // 根据静止密度和粒子体积计算新的质量

    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1000.0f);
    nh.param("sph/gasConstant", gasConstant, 1.0f);
    nh.param("sph/viscosity", viscosity, 1.04f);
    nh.param("sph/h", h, 0.15f);//这个参数很影响，无法与实际对应，思考这个问题咋办
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/tension", tension, 0.2f);
    nh.param("sph/use_pctrl", use_pctrl, true);

    // nh.param("mass", mass, new_mass);
    // nh.param("restDensity", restDensity, 1000.0f);
    // nh.param("gasConstant", gasConstant, 1.0f);
    // nh.param("viscosity", viscosity, 1.04f); // 可能需要根据模拟结果进一步调整
    // nh.param("h", h, new_h);
    // nh.param("g", g, -9.8f); // 根据新的h调整重力作用范围可能需要更细致的分析
    // nh.param("tension", tension, 0.2f); // 表面张力参数也可能需要调整
    // nh.param("use_pctrl", use_pctrl, true);

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    // wait 3 seconds
    ros::Duration(3.0).sleep();

    //ros sub&pub
    odomBroadcast_sub     = nh.subscribe("/odomBroadcast", 1000,  odomBroadcastCallback);
    timer                 = nh.createTimer(ros::Duration(0.05),   timerCallback);
    nav_goal_sub          = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
    particles_publisher   = nh.advertise<visualization_msgs::Marker>("particles_vis", 10);
    virtual_particles_vis = nh.advertise<visualization_msgs::Marker>("virtual_particles_vis", 10);
    // traj_sub              = nh.subscribe("/bspline_traj", 10, trajCallback);
    //create parallel sub
    for (const auto &info : master_topics) {
            // Check if topic name matches the pattern /uav(i)_odomWithNeighbors
            std::regex topic_pattern("/uav\\d+_odomWithNeighbors");
            if (std::regex_match(info.name, topic_pattern)) {
                // every uav has a buffer to store the odom_withNeighbors.   
                subscribeOdomWithNeighbors(info.name, nh);

            // Extract UAV name and create a publisher for it
            std::string uav_name = extractUavName(info.name);
            if (!uav_name.empty()) {
                publishPositionCommand(uav_name, nh);
            }
        }    
    }

    //start sph planner
    SPHSettings sphSettings(mass, restDensity, gasConstant, viscosity, h, g, tension);
    // SPHSettings sphSettings(10000.0, restDensity, gasConstant, viscosity, 5.0, g, tension);
	
    sph_planner = new SPHSystem(15, sphSettings, false);
    // sph_planner ->initPlanner();//no need

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
}

void publishPositionCommand(const std::string& uav_name, ros::NodeHandle& nh) {
    std::string topic_name = uav_name + "/position_cmd";
    uav_publishers[uav_name] = nh.advertise<quadrotor_msgs::PositionCommand>(topic_name, 10);
}


void odomWithNeighborsCallback(const water_swarm::OdomWithNeighborsConstPtr& msg, const std::string& uav_name) {
    odomWithNeighbors[extractUavName(uav_name)] = *msg;
}

void subscribeOdomWithNeighbors(const std::string &topic_name, ros::NodeHandle &nh) {
    odomSubscribers[topic_name] = nh.subscribe<water_swarm::OdomWithNeighbors>(
        topic_name, 
        1000, 
        boost::bind(odomWithNeighborsCallback, _1, topic_name)
    );
}

void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sph_planner->started = true;
    ROS_INFO("Received 2D Nav Goal at position: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    // 
}

void timerCallback(const ros::TimerEvent&) {
    // ROS_INFO("Timer triggered");
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
        // 计算 deltaTime，如果是第一次进入，则将其设置为0
        float deltaTime = last_time.isZero() ? 0.0 : (current_time - last_time).toSec();

        // 更新last_time为当前时间
        last_time = current_time;

        sph_planner->update(deltaTime);
    }
}

void odomBroadcastCallback(const water_swarm::OdomBroadcast::ConstPtr& msg) {
    if (!isInitialReceived) {
        initial_odomBroadcast_ = *msg;
        isInitialReceived = true;

        //init planner 
        sph_planner->initPlanner();
        
        // ROS_INFO("Initial OdomBroadcast received.");
    } 
    else {
        current_odomBroadcast_ = *msg;
        // ROS_INFO("Current OdomBroadcast updated.");
    }
}

void trajCallback(const bspline_race::BsplineTraj::ConstPtr& msg) {

    if (first_traj)
    {
        // 清空旧数据
        global_positions.clear();
        global_velocities.clear();
        global_accelerations.clear();

        // 存储新数据
        global_positions     = msg->position;
        global_velocities    = msg->velocity;
        global_accelerations = msg->acceleration;

        first_traj = false;        
    }
}

SPHSystem::SPHSystem(
    size_t particleCubeWidth, const SPHSettings &settings,
    const bool &runOnGPU)
    : particleCubeWidth(particleCubeWidth)
    , settings(settings)
    , runOnGPU(runOnGPU)
{
    // // particleCount = particleCubeWidth * particleCubeWidth * particleCubeWidth;
    // particleCount = initial_odomBroadcast_.OdomBroadcast.size();
    // // std::cout<< particleCount <<std::endl;
    // particles = (Particle*)malloc(sizeof(Particle) * particleCount);

    // initParticles();

	//if start init
	// started = false;
}

SPHSystem::SPHSystem()
    : particleCubeWidth(0.0),
      settings(),
      started(false),
      runOnGPU(false){}

SPHSystem::~SPHSystem() {
    delete[](particles);
}

void SPHSystem::initPlanner()
{
    // particleCount = particleCubeWidth * particleCubeWidth * particleCubeWidth;
    particleCount = initial_odomBroadcast_.OdomBroadcast.size();
    std::cout<< particleCount <<std::endl;
    particles = (Particle*)malloc(sizeof(Particle) * particleCount);

    initParticles();

    //calc boundary 
    particlessideLength =   std::ceil(std::sqrt(particleCount)) + 1;
    particlesPerSide    =   particlessideLength * 10;
    water_swarm::Position apex;
    apex.x = -0.5; apex.y = -0.5; apex.z = 0;
    generateVirtualParticles(particlessideLength, particlesPerSide, apex);
	
    //start init
	started = false;
}

void SPHSystem::initParticles()
{
    for (size_t i = 0; i < particleCount; i++)
    {
        Particle* particle = &particles[i];
        particle->name     = initial_odomBroadcast_.drone_names[i];
        particle->position = initial_odomBroadcast_.OdomBroadcast[i].position;
        particle->velocity = initial_odomBroadcast_.OdomBroadcast[i].velocity;
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
    Particle *particles, const size_t particleCount, const SPHSettings &settings,
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
    Particle *particles, const size_t particleCount, const SPHSettings &settings,
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

    // ROS_INFO("UPDATE!!!");
}

void SPHSystem::parallelDensityAndPressures()
{   
    float massPoly6Product = settings.mass * settings.poly6;
    for (size_t i = 0; i < particleCount; i++) {
    Particle& pi = particles[i];

        // 使用at方法安全访问map元素
        try {
            const auto& odomNeighbors = odomWithNeighbors.at(pi.name.data);

            float pDensity = 0;

            for (const auto& neighborOdom : odomNeighbors.neighborsOdom) {
                water_swarm::Position diff;
                diff.x = neighborOdom.position.x - odomNeighbors.myOdom.position.x;
                diff.y = neighborOdom.position.y - odomNeighbors.myOdom.position.y;
                diff.z = neighborOdom.position.z - odomNeighbors.myOdom.position.z;

                float dist2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
                if (dist2 < settings.h2) {
                    pDensity += massPoly6Product * std::pow(settings.h2 - dist2, 3);
                }
            }

            // 包括自身的密度
            pi.density = pDensity + settings.selfDens;
            std::cout<< "partice "<<i<<" density is: "<<  pi.density << 
                    "  neighbors num is: " << odomNeighbors.neighborsOdom.size() << std::endl;
            // 计算压力
            pi.pressure = settings.gasConstant * (pi.density - settings.restDensity);

        } catch (const std::out_of_range& e) {
            // 处理键不存在的情况，例如打印错误消息或跳过当前迭代
            std::cerr << "Error: Particle name not found in odomWithNeighbors map." << std::endl;
            continue;
        }
    }
}

void SPHSystem::parallelForces()
{
    for (size_t i = 0; i < particleCount; i++) {

        Particle& pi = particles[i];
        // 重置力
        pi.force.x = 0.0;
        pi.force.y = 0.0;
        pi.force.z = 0.0;

        auto it = odomWithNeighbors.find(pi.name.data);
        if (it == odomWithNeighbors.end()) continue;

        const auto& myOdom        = it->second.myOdom;
        const auto& neighborsOdom = it->second.neighborsOdom;
        const auto& drone_names   = it->second.drone_names;

        for (size_t j = 0; j < neighborsOdom.size(); ++j) {
            const auto& neighborOdom = neighborsOdom[j];
            const auto& pj_name = drone_names[j]; // 获取当前邻居的名字

            Particle* pj = nullptr; // 指向找到的粒子
            for (size_t k = 0; k < particleCount; ++k) {
                if (particles[k].name == pj_name) {
                    pj = &particles[k];
                    break; // 找到匹配的粒子，跳出循环
                }
            }
            if (!pj) {
                continue; // 如果没有找到对应的粒子，继续下一个循环
            }

            // 计算距离和方向
            auto dx = neighborOdom.position.x - myOdom.position.x;
            auto dy = neighborOdom.position.y - myOdom.position.y;
            auto dz = neighborOdom.position.z - myOdom.position.z;
            double dist2 = dx * dx + dy * dy + dz * dz;

            if (dist2 < settings.h2 && dist2 > 0) {
                double dist = std::sqrt(dist2);

                //计算归一化向量
                double dir_x = dx / dist;
                double dir_y = dy / dist;
                double dir_z = dz / dist;

                // 计算压力力，使用邻居粒子的压力和密度
                double pressure = (pi.pressure + pj->pressure) / (2.0 * pj->density);
                
                water_swarm::Force pressureForce;
                pressureForce.x = -dir_x * settings.mass * pressure * settings.spikyGrad;
                pressureForce.y = -dir_y * settings.mass * pressure * settings.spikyGrad;
                pressureForce.z = -dir_z * settings.mass * pressure * settings.spikyGrad;
                pressureForce.x *= std::pow(settings.h - dist, 2);
                pressureForce.y *= std::pow(settings.h - dist, 2);
                pressureForce.z *= std::pow(settings.h - dist, 2);
                
                // 计算粘性力
                auto dvx = pj->velocity.x - myOdom.velocity.x;
                auto dvy = pj->velocity.y - myOdom.velocity.y;
                auto dvz = pj->velocity.z - myOdom.velocity.z;

                water_swarm::Force viscosityForce; 
                viscosityForce.x = settings.mass * settings.viscosity * dvx / pj->density * settings.spikyLap * (settings.h - dist);
                viscosityForce.y = settings.mass * settings.viscosity * dvy / pj->density * settings.spikyLap * (settings.h - dist);
                viscosityForce.z = settings.mass * settings.viscosity * dvz / pj->density * settings.spikyLap * (settings.h - dist);

                pi.force.x += pressureForce.x + viscosityForce.x;
                pi.force.y += pressureForce.y + viscosityForce.y;
                pi.force.z += pressureForce.z + viscosityForce.z;
            }
        }
    }
}

void SPHSystem::calaDynamicBound()
{

}


// void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
// {
//     if (!global_accelerations.empty()) {
//         // 获取轨迹的第一个加速度
//         auto& traj_acceleration = global_accelerations.front().pose.position;

//         for (size_t i = 0; i < particleCount; i++) {
//             Particle *p = &particles[i];

//             // 计算加速度和速度
//             water_swarm::Acceleration acceleration;
//             acceleration.x = p->force.x / p->density; //+ traj_acceleration.x;
//             acceleration.y = p->force.y / p->density; //+ traj_acceleration.y;
//             acceleration.z = p->force.z / p->density;  // 不改变Z方向的加速度

//             p->velocity.x += acceleration.x * deltaTime;
//             p->velocity.y += acceleration.y * deltaTime;
//             p->velocity.z += acceleration.z * deltaTime;
            
//             // 预计算新位置
//             double newX = p->position.x + p->velocity.x * deltaTime;
//             double newY = p->position.y + p->velocity.y * deltaTime;
//             double newZ = p->position.z + p->velocity.z * deltaTime;

//             // 检查是否触碰到虚拟粒子构成的墙
//             if (isNearVirtualParticle(newX, newY, newZ)) {
//                 // 碰撞反弹，这里简化处理，仅在x和y方向反弹
//                 p->velocity.x = -p->velocity.x;
//                 p->velocity.y = -p->velocity.y;
//             }

//             // 更新位置，碰撞处理后
//             p->position.x += p->velocity.x * deltaTime;
//             p->position.y += p->velocity.y * deltaTime;
//             p->position.z += p->velocity.z * deltaTime;
//         }

//         // 删除已应用的轨迹加速度
//         global_accelerations.erase(global_accelerations.begin());
//     }
// }

void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{

    for (size_t i = 0; i < particleCount; i++) {
        Particle *p = &particles[i];

        // 计算加速度和速度
        water_swarm::Acceleration acceleration;
        acceleration.x = p->force.x / p->density; //+ traj_acceleration.x;
        acceleration.y = p->force.y / p->density; //+ traj_acceleration.y;
        acceleration.z = p->force.z / p->density;  // 不改变Z方向的加速度

        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;
        
        // 预计算新位置
        double newX = p->position.x + p->velocity.x * deltaTime;
        double newY = p->position.y + p->velocity.y * deltaTime;
        double newZ = p->position.z + p->velocity.z * deltaTime;

        // 检查是否触碰到虚拟粒子构成的墙
        if (isNearVirtualParticle(newX, newY, newZ)) {
            // 碰撞反弹，这里简化处理，仅在x和y方向反弹
            p->velocity.x = -p->velocity.x;
            p->velocity.y = -p->velocity.y;
        }

        // 更新位置，碰撞处理后
        p->position.x += p->velocity.x * deltaTime;
        p->position.y += p->velocity.y * deltaTime;
        p->position.z += p->velocity.z * deltaTime;
    }

}

void SPHSystem::pubroscmd()
{   
    // 初始化visualization消息
    visualization_msgs::Marker points;
    points.header.frame_id = "world";  // 修改为适当的frame ID
    points.header.stamp = ros::Time::now();
    points.ns = "sph_system";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.5; //
    points.scale.y = 0.5; // 
    points.color.g = 1.0f;
    points.color.a = 1.0;

    visualization_msgs::Marker virtual_points; // 虚拟粒子
    virtual_points.header.frame_id = "world";
    virtual_points.header.stamp = ros::Time::now();
    virtual_points.ns = "virtual_particles";
    virtual_points.action = visualization_msgs::Marker::ADD;
    virtual_points.pose.orientation.w = 1.0;
    virtual_points.id = 0;
    virtual_points.type = visualization_msgs::Marker::POINTS;
    virtual_points.scale.x = 0.2; // 
    virtual_points.scale.y = 0.2; // 
    virtual_points.color.r = 0.5f;
    virtual_points.color.g = 0.0f;
    virtual_points.color.b = 0.5f;
    virtual_points.color.a = 1.0; //

    //根据Particle的名字更新位置
    for (size_t i = 0; i < particleCount; i++) {
        Particle& p = particles[i];

        // 创建一个新的点，并将其添加到点数组中
        geometry_msgs::Point point;
        point.x = p.position.x;
        point.y = p.position.y;
        point.z = p.position.z;
        points.points.push_back(point);

        auto it = uav_publishers.find(p.name.data); // 假设name字段是std_msgs::String类型
        if (it != uav_publishers.end()) {
            quadrotor_msgs::PositionCommand cmd_msg;
            cmd_msg.header.stamp = ros::Time::now();
            cmd_msg.yaw = 0.0;
            cmd_msg.yaw_dot = 0.0;

            if (use_pctrl) {
                cmd_msg.position.x = p.position.x;
                cmd_msg.position.y = p.position.y;
                cmd_msg.position.z = p.position.z;
            }

            if (use_vctrl) {
                cmd_msg.velocity.x = p.velocity.x;
                cmd_msg.velocity.y = p.velocity.y;
                cmd_msg.velocity.z = p.velocity.z;
                cmd_msg.position.x = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.position.y = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.position.z = std::numeric_limits<float>::quiet_NaN();
            }

            if (use_actrl) {
                cmd_msg.acceleration.x = p.acceleration.x;
                cmd_msg.acceleration.y = p.acceleration.y;
                cmd_msg.acceleration.z = p.acceleration.z;
                cmd_msg.velocity.x = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.velocity.y = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.velocity.z = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.position.x = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.position.y = std::numeric_limits<float>::quiet_NaN();
                cmd_msg.position.z = std::numeric_limits<float>::quiet_NaN();
            }

            it->second.publish(cmd_msg);
        }
    }

    for (const auto& particle : virtual_particles) {
        geometry_msgs::Point p;
        p.x = particle.position.x;
        p.y = particle.position.y;
        p.z = particle.position.z;
        virtual_points.points.push_back(p);
    }

    particles_publisher.publish(points);
    virtual_particles_vis.publish(virtual_points);
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