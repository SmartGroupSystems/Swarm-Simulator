#!/bin/bash

# 启动 ROS 主节点
roscore &
sleep 2  # 等待一段时间以确保 roscore 完全启动

# 定义 UAVs 的数量
NUM_UAVS=$1

# 计算正方形边长，确保所有无人机可以被包含在这个正方形内
SIDE_LENGTH=$(echo "sqrt($NUM_UAVS)" | bc)
if [ $(echo "$SIDE_LENGTH * $SIDE_LENGTH" | bc) -lt $NUM_UAVS ]; then
    SIDE_LENGTH=$(($SIDE_LENGTH + 1))
fi

# # 通过 ((SIDE_LENGTH / 2)) 计算正方形一半的长度
# half_length=$((SIDE_LENGTH / 2))

# 生成初始位置坐标
INIT_POSITIONS=()

for ((x=0; x<$SIDE_LENGTH; x++))
do
    for ((y=0; y<$SIDE_LENGTH; y++))
    do
        if [ ${#INIT_POSITIONS[@]} -lt $NUM_UAVS ]; then
            INIT_POSITIONS+=("$x $y 1")
        else
            break
        fi
    done
done

# # 计算正方形边长
# SIDE_LENGTH=$(echo "scale=0; sqrt($NUM_UAVS)" | bc)
# if [ $(echo "$SIDE_LENGTH * $SIDE_LENGTH" | bc) -lt $NUM_UAVS ]; then
#     SIDE_LENGTH=$(($SIDE_LENGTH + 1))
# fi

# # 通过 ((SIDE_LENGTH / 2)) 计算正方形一半的长度
# half_length=$((SIDE_LENGTH / 2))

# # 生成初始位置坐标
# INIT_POSITIONS=()

# # 使用一个嵌套循环遍历从 -half_length 到 half_length 的每个位置
# for ((x=-half_length; x<half_length; x++))
# do
#     for ((y=-half_length; y<half_length; y++))
#     do
#         if [ ${#INIT_POSITIONS[@]} -lt $NUM_UAVS ]; then
#             INIT_POSITIONS+=("$x $y 1")
#         else
#             break 2 # 当达到无人机数上限时，完全退出所有循环
#         fi
#     done
# done

# 循环来启动每个 UAV
for ((i=1; i<=NUM_UAVS; i++))
do
    # 解析初始位置
    POSITION=(${INIT_POSITIONS[$((i-1))]})
    INIT_X=${POSITION[0]}
    INIT_Y=${POSITION[1]}
    INIT_Z=${POSITION[2]}

    # 指定命名空间并启动模拟器
    ROS_NAMESPACE=uav$i roslaunch so3_quadrotor_simulator simulator.launch init_x:=$INIT_X init_y:=$INIT_Y init_z:=$INIT_Z namespace:=uav$i &
done

# 启动 RVIZ
rosrun rviz rviz -d $(rospack find so3_quadrotor_simulator)/config/rviz.rviz
