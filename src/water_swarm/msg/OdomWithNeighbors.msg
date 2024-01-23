# OdomWithNeighbors.msg
std_msgs/Header header  # 时间戳和帧编号
Odom myOdom            # 自定义的Odom类型
Odom[] neighborsOdom   # 一个Odom数组，用于存储邻居的Odom信息
