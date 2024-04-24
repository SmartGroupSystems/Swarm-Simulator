#!/bin/bash

source ~/water_swarm/devel/setup.bash  

echo "Starting multi-quadrotor simulator with drones..."
./multi_quadrotor_simulator.sh 25 & sleep 10;

echo "Launching bspline_race planning..."
roslaunch bspline_race planning.launch & sleep 2;

echo "Running water_swarm neighbors node..."
rosrun water_swarm neighbors


