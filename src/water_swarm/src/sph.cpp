#include "sph.h"

SPHSystem sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_planner");
    ros::NodeHandle nh("~");
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    //ros sub&pub
    odomBroadcast_sub   = nh.subscribe("/odomBroadcast", 1000,  odomBroadcastCallback);
    timer               = nh.createTimer(ros::Duration(0.05),   timerCallback);
    nav_goal_sub        = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
    
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

    sph_planner.initPlanner();

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
}

void publishPositionCommand(const std::string& uav_name, ros::NodeHandle& nh) {
    std::string topic_name = uav_name + "/position_cmd";
    uav_publishers[uav_name] = nh.advertise<quadrotor_msgs::PositionCommand>(topic_name, 10);
}


void odomWithNeighborsCallback(const water_swarm::OdomWithNeighborsConstPtr& msg, const std::string& uav_name) {
    odomWithNeighbors[uav_name] = *msg;
}

void subscribeOdomWithNeighbors(const std::string &topic_name, ros::NodeHandle &nh) {
    odomSubscribers[topic_name] = nh.subscribe<water_swarm::OdomWithNeighbors>(
        topic_name, 
        1000, 
        boost::bind(odomWithNeighborsCallback, _1, topic_name)
    );
}

void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sph_planner.started = true;
    ROS_INFO("Received 2D Nav Goal at position: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    // 
}

void timerCallback(const ros::TimerEvent&) {
    // ROS_INFO("Timer triggered");
    if (!sph_planner.started) 
    {
        ROS_INFO("Do not receive the start command, return.");
        return;
    }
    else
    {   
        current_time = ros::Time::now();

        // 计算 deltaTime，如果是第一次进入，则将其设置为0
        float deltaTime = last_time.isZero() ? 0.0 : (current_time - last_time).toSec();

        // 更新last_time为当前时间
        last_time = current_time;

        sph_planner.update(deltaTime);
    }
}

void odomBroadcastCallback(const water_swarm::OdomBroadcast::ConstPtr& msg) {
    if (!isInitialReceived) {
        initial_odomBroadcast_ = *msg;
        isInitialReceived = true;
        // ROS_INFO("Initial OdomBroadcast received.");
    } else {
        current_odomBroadcast_ = *msg;
        // ROS_INFO("Current OdomBroadcast updated.");
    }
}

SPHSystem::SPHSystem(
    size_t particleCubeWidth, const SPHSettings &settings,
    const bool &runOnGPU)
    : particleCubeWidth(particleCubeWidth)
    , settings(settings)
    , runOnGPU(runOnGPU)
{
    // particleCount = particleCubeWidth * particleCubeWidth * particleCubeWidth;
    particleCount = initial_odomBroadcast_.OdomBroadcast.size();
    particles = (Particle*)malloc(sizeof(Particle) * particleCount);

    initParticles();

	//start init
	started = false;
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
    particles = (Particle*)malloc(sizeof(Particle) * particleCount);

    initParticles();

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

            const auto& myOdom = it->second.myOdom;
            for (const auto& neighborOdom : it->second.neighborsOdom) {
                // 计算距离和方向
                auto dx = neighborOdom.position.x - myOdom.position.x;
                auto dy = neighborOdom.position.y - myOdom.position.y;
                auto dz = neighborOdom.position.z - myOdom.position.z;
                double dist2 = dx*dx + dy*dy + dz*dz;

                if (dist2 < settings.h2 && dist2 > 0) {
                    double dist = std::sqrt(dist2);
                    double h_minus_dist = settings.h - dist;

                    // 计算压力力
                    double pressure = (pi.pressure + pi.pressure) / (2.0 * pi.density); // 假设pi和邻居的密度相似
                    water_swarm::Force pressureForce;
                    pressureForce.x =    -pressure * dx / dist * h_minus_dist * h_minus_dist;
                    pressureForce.y =    -pressure * dy / dist * h_minus_dist * h_minus_dist;
                    pressureForce.z =    -pressure * dz / dist * h_minus_dist * h_minus_dist;

                    // 计算粘性力
                    auto dvx = neighborOdom.velocity.x - myOdom.velocity.x;
                    auto dvy = neighborOdom.velocity.y - myOdom.velocity.y;
                    auto dvz = neighborOdom.velocity.z - myOdom.velocity.z;
                    water_swarm::Force viscosityForce; 
                    viscosityForce.x =    settings.viscosity * dvx / pi.density * h_minus_dist;
                    viscosityForce.y =    settings.viscosity * dvy / pi.density * h_minus_dist;
                    viscosityForce.z =    settings.viscosity * dvz / pi.density * h_minus_dist;

                    // 应用力
                    pi.force.x += (pressureForce.x + viscosityForce.x) * settings.mass;
                    pi.force.y += (pressureForce.y + viscosityForce.y) * settings.mass;
                    pi.force.z += (pressureForce.z + viscosityForce.z) * settings.mass;
                }
            }
    }
}

void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particleCount; i++) {
        Particle *p = &particles[i];
        
        // 计算加速度和速度
        water_swarm::Acceleration acceleration;
        acceleration.x = p->force.x / p->density;
        acceleration.y = p->force.y / p->density; 
        acceleration.z = p->force.z / p->density + settings.g; // 假设settings.g是重力加速度在z方向的分量;

        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;

        // 更新位置
        p->position.x += p->velocity.x * deltaTime;
        p->position.y += p->velocity.y * deltaTime;
        p->position.z += p->velocity.z * deltaTime;
    }
}

void SPHSystem::pubroscmd()
{
    //根据Particle的名字更新位置
    for (size_t i = 0; i < particleCount; i++) {
        Particle& p = particles[i];
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
    
}