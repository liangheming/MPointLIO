# PoinLIO

## 主要工作
1. 该repo主要参考论文[PointLIO](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)实现，与官方代码存在较大差异；
2. 目前主要支持mid360，支持较高的lidar采集频率(10-100hz)；
3. 目前仅支LIDAR+IMU(IMU作为观测，更新状态量)的方式；

## 环境依赖
1. ubuntu 20.04
2. ros neotic

## 编译环境依赖
- Eigen
- pcl
- LIVOX-SDK2
- livox_ros_driver2

### 编译及安装
#### 1. Eigen && pcl (ubuntu20.04 系统自带的即可)
```shell
sudo apt install libpcl-dev libeigen3-dev
```
#### 2. LIVOX-SDK2 && livox_ros_driver2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

#### 3. PointLIO
```shell
git clone https://github.com/liangheming/MPointLIO.git
source ${livox_ros_driver2的安装目录}/devel/setup.bash
cd MPointLIO && catkin_make
```

### 基础运行
1. 启动里程计节点并播包
```shell
roslaunch point_lio point_mid360.launch
rosbag play your.bag
```
2. 基于mid360实时进行里程计
```shell
roslaunch livox_ros_driver2 msg_MID360.launch publish_freq:=100
roslaunch point_lio point_mid360.launch
```

### 一些实现说明
1. PointLIO本质上来一个点就进行一次预测和更新，理论上频率和点(一个点而不是点云)采集频率一致；
2. 对于一般的sdk如Livox-SDK2，往往累积一帧点云后才发布，所以实际运行时最高频率为min(imu_hz,lidar_hz);
3. 该repo以点云的频率为主，然后在一帧点云之间插入imu数据的方式进行数据同步，所以最高频率是lidar的频率(mid360的sdk目前最高100hz)

### 测试数据（100hz的mid360点云）
```text
链接: https://pan.baidu.com/s/1MgnHttpSgqGySmz8GUhmFg?pwd=1mgw 提取码: 1mgw 
--来自百度网盘超级会员v7的分享
```

### 参考及致谢
1. [PointLIO论文](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)
2. [PointLIO官方代码](https://github.com/hku-mars/Point-LIO)
