#pragma once

#include <chrono>
#include <queue>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include "commons.h"
#include "map_builder.h"

struct NodeConfig
{
    std::string imu_topic;
    std::string lidar_topic;
    double imu_acc_multi = 10.0;
    double imu_gyro_multi = 1.0;
    int filter_num = 6;
    double lidar_scan_min_range = 0.25;
    double lidar_scan_max_range = 30.0;
    std::string map_frame;
    std::string body_frame;
};

struct NodeState
{
    boost::mutex imu_mutex;
    boost::mutex lidar_mutex;
    std::deque<IMUDate> imu_queue;
    std::deque<std::pair<double, CloudType::Ptr>> lidar_queue;
    double last_imu_time = 0.0;
    double last_lidar_start_time = 0.0;
    double last_lidar_end_time = 0.0;
    bool is_activate = true;
    bool lidar_pushed = false;
};

class PointLIOROS
{
public:
    PointLIOROS(ros::NodeHandle &nh);
    void loadParameters();
    void initSubscribers();
    void initPublishers();
    void threadCallBack();

    void lidarCallback(const livox_ros_driver2::CustomMsgConstPtr &msg);
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    bool syncPackage();
    void broadcastTf();
    void publishOdom();

    ~PointLIOROS();

private:
    ros::NodeHandle m_nh;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_lidar_sub;
    ros::Publisher m_point_pub;
    ros::Publisher m_odom_pub;
    boost::thread m_main_thread;
    Package m_package;
    NodeState m_state;
    NodeConfig m_config;
    MapBuilderConfig m_map_builder_config;
    std::shared_ptr<MapBuilder> m_map_builder;
    pcl::VoxelGrid<PointType> m_voxel_filter;
};