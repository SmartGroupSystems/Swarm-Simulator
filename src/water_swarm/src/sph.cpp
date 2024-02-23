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
            // Check if topic name matches the pattern /uav(i)/sim/odom
            std::regex topic_pattern("/uav\\d+_odomWithNeighbors");
            if (std::regex_match(info.name, topic_pattern)) {
                // every uav has a buffer to store the odom_withNeighbors.   
                subscribeOdomWithNeighbors(info.name, nh);
            }  
    }

    sph_planner.initPlanner();

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
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
    {   /// Here deltaTime is no use, just satisfy the func...
        float deltaTime;
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
	// To increase system stability, a fixed deltaTime is set
	deltaTime = 0.003f;
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
    parallelUpdateParticlePositions();

    //rospub control commands

}

void SPHSystem::parallelDensityAndPressures()
{   
    float massPoly6Product = settings.mass * settings.poly6;
    // use chatgpt rebuild...
    
}

void SPHSystem::parallelForces()
{

}

void SPHSystem::parallelUpdateParticlePositions()
{

}