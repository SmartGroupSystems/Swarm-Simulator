#include "sph.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_planner");
    ros::NodeHandle nh("~");

    nh.param("mass", mass, 1.00f);
    nh.param("restDensity", restDensity, 1000.0f);
    nh.param("gasConstant", gasConstant, 1.0f);
    nh.param("viscosity", viscosity, 1.04f);
    nh.param("h", h, 0.15f);//这个参数很影响，无法与实际对应，思考这个问题咋办
    nh.param("g", g, -9.8f);
    nh.param("tension", tension, 0.2f);
    nh.param("use_pctrl", use_pctrl, true);
    
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    //ros sub&pub
    odomBroadcast_sub   = nh.subscribe("/odomBroadcast", 1000,  odomBroadcastCallback);
    timer               = nh.createTimer(ros::Duration(0.05),   timerCallback);
    nav_goal_sub        = nh.subscribe("/move_base_simple/goal", 10, navGoalCallback);
    particles_publisher = nh.advertise<visualization_msgs::Marker>("particles_vis", 10);

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

    // wait 2 seconds
    ros::Duration(2.0).sleep();

    //start sph planner
    SPHSettings sphSettings(mass, restDensity, gasConstant, viscosity, h, g, tension);
	
    sph_planner = new SPHSystem(15, sphSettings, false);
    // sph_planner ->initPlanner();

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


void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particleCount; i++) {
        Particle *p = &particles[i];
        
        // 计算加速度和速度
        water_swarm::Acceleration acceleration;
        acceleration.x = p->force.x / p->density;
        acceleration.y = p->force.y / p->density; 
        // acceleration.z = p->force.z / p->density + settings.g; // 假设settings.g是重力加速度在z方向的分量;
        acceleration.z = p->force.z / p->density;

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
    // 初始化visualization消息
    visualization_msgs::Marker points;
    points.header.frame_id = "world";  // 修改为适当的frame ID
    points.header.stamp = ros::Time::now();
    points.ns = "sph_system";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置点的尺寸
    points.scale.x = 0.5; // 直径为0.5
    points.scale.y = 0.5; // 直径为0.5
    points.color.g = 1.0f;
    points.color.a = 1.0;

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
    particles_publisher.publish(points);
}