#include "point_lio_ros.h"

PointLIOROS::PointLIOROS(ros::NodeHandle &nh)
    : m_nh(nh)
{
    loadParameters();
    initSubscribers();
    initPublishers();
    m_voxel_filter.setLeafSize(m_map_builder_config.scan_resolution, m_map_builder_config.scan_resolution, m_map_builder_config.scan_resolution);
    m_map_builder = std::make_shared<MapBuilder>(m_map_builder_config);
    m_main_thread = boost::thread(boost::bind(&PointLIOROS::threadCallBack, this));
}

void PointLIOROS::loadParameters()
{
    m_nh.param<std::string>("imu_topic", m_config.imu_topic, "/livox/imu");
    m_nh.param<std::string>("lidar_topic", m_config.lidar_topic, "/livox/lidar");
    m_nh.param<std::string>("map_frame", m_config.map_frame, "lidar");
    m_nh.param<std::string>("body_frame", m_config.body_frame, "body");
    m_nh.param<int>("filter_num", m_config.filter_num, 6);
    m_nh.param<double>("lidar_scan_min_range", m_config.lidar_scan_min_range, 0.25);
    m_nh.param<double>("lidar_scan_max_range", m_config.lidar_scan_max_range, 30.0);

    m_nh.param<double>("imu_acc_multi", m_config.imu_acc_multi, 10.0);
    m_nh.param<double>("imu_gyro_multi", m_config.imu_gyro_multi, 1.0);

    m_nh.param<int>("imu_init_num", m_map_builder_config.imu_init_num, 100);
    m_nh.param<double>("vel_cov", m_map_builder_config.vel_cov, 20.0);
    m_nh.param<double>("acc_cov", m_map_builder_config.acc_cov, 500);
    m_nh.param<double>("gyro_cov", m_map_builder_config.gyro_cov, 1000);
    m_nh.param<double>("b_acc_cov", m_map_builder_config.b_acc_cov, 0.0001);
    m_nh.param<double>("b_gyro_cov", m_map_builder_config.b_gyro_cov, 0.0001);
    m_nh.param<double>("lidar_meas_cov_inv", m_map_builder_config.lidar_meas_cov_inv, 100);
    m_nh.param<double>("imu_meas_acc_cov_inv", m_map_builder_config.imu_meas_acc_cov_inv, 10);
    m_nh.param<double>("imu_meas_gyro_cov_inv", m_map_builder_config.imu_meas_gyro_cov_inv, 10);
    m_nh.param<double>("satu_acc", m_map_builder_config.satu_acc, 20.0);
    m_nh.param<double>("satu_gyro", m_map_builder_config.satu_gyro, 35);
    m_nh.param<double>("scan_resolution", m_map_builder_config.scan_resolution, 0.1);
    m_nh.param<double>("map_resolution", m_map_builder_config.map_resolution, 0.3);
    m_nh.param<double>("near_search_num", m_map_builder_config.near_search_num, 5);
    m_nh.param<double>("cube_len", m_map_builder_config.cube_len, 300.0);
    m_nh.param<double>("move_thresh", m_map_builder_config.move_thresh, 1.5);
    m_nh.param<double>("det_range", m_map_builder_config.det_range, 60.0);
}

void PointLIOROS::initSubscribers()
{
    m_imu_sub = m_nh.subscribe(m_config.imu_topic, 100, &PointLIOROS::imuCallback, this);
    m_lidar_sub = m_nh.subscribe(m_config.lidar_topic, 100, &PointLIOROS::lidarCallback, this);
}

void PointLIOROS::initPublishers()
{
    m_point_pub = m_nh.advertise<sensor_msgs::PointCloud2>("body_cloud", 100);
    m_odom_pub = m_nh.advertise<nav_msgs::Odometry>("odom", 100);
}

void PointLIOROS::lidarCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
{
    CloudType::Ptr cloud = livox2PCL(msg, m_config.filter_num, m_config.lidar_scan_min_range, m_config.lidar_scan_max_range, m_state.last_lidar_end_time);
    {
        boost::lock_guard<boost::mutex> lock(m_state.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < m_state.last_lidar_start_time)
        {
            ROS_WARN("Lidar message is not in order:%f %f", timestamp, m_state.last_lidar_start_time);
            std::deque<std::pair<double, CloudType::Ptr>>().swap(m_state.lidar_queue);
        }
        m_state.lidar_queue.emplace_back(timestamp, cloud);
        m_state.last_lidar_start_time = timestamp;
        m_state.last_lidar_end_time = (m_state.last_lidar_start_time + cloud->points.back().curvature);
    }
}

void PointLIOROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    {
        boost::lock_guard<boost::mutex> lock(m_state.imu_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < m_state.last_imu_time)
        {
            ROS_WARN("IMU message is not in order:%f %f", timestamp, m_state.last_imu_time);
            std::deque<IMUDate>().swap(m_state.imu_queue);
        }
        m_state.imu_queue.emplace_back(timestamp,
                                       msg->linear_acceleration.x * m_config.imu_acc_multi,
                                       msg->linear_acceleration.y * m_config.imu_acc_multi,
                                       msg->linear_acceleration.z * m_config.imu_acc_multi,
                                       msg->angular_velocity.x * m_config.imu_gyro_multi,
                                       msg->angular_velocity.y * m_config.imu_gyro_multi,
                                       msg->angular_velocity.z * m_config.imu_gyro_multi);
        m_state.last_imu_time = timestamp;
    }
}

void PointLIOROS::threadCallBack()
{
    while (ros::ok() && m_state.is_activate)
    {
        if (!syncPackage())
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            continue;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        m_map_builder->process(m_package);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double millisecond = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;
        ROS_DEBUG("Process package:%f ms", millisecond);

        if (m_map_builder->status() != MapBuilder::Status::MAPPING)
            continue;

        broadcastTf();
        publishOdom();

        if (m_package.lidar_end_time - m_map_builder->lastMapUpdateTime() > 0.1)
        {
            m_map_builder->trimMap();
            Quatd q = m_map_builder->eskf.x.q.inverse();
            Vec3d p = q * m_map_builder->eskf.x.p * (-1.0);
            CloudType::Ptr cloud(new CloudType);
            pcl::transformPointCloud(*m_map_builder->cacheClouds(), *cloud, p, q);
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.frame_id = m_config.body_frame;
            msg.header.stamp = ros::Time(m_map_builder->lastMapUpdateTime());
            m_point_pub.publish(msg);
            m_map_builder->increMap();
        }
    }
}

bool PointLIOROS::syncPackage()
{
    if (m_state.lidar_queue.empty() || m_state.imu_queue.empty())
        return false;
    if (!m_state.lidar_pushed)
    {
        m_package.cloud_in = m_state.lidar_queue.front().second;
        // m_voxel_filter.setInputCloud(m_package.cloud_in);
        // m_voxel_filter.filter(*m_package.cloud_in);
        m_package.lidar_start_time = m_state.lidar_queue.front().first;
        m_package.first_point_time = m_package.lidar_start_time + m_package.cloud_in->points.front().curvature;
        m_package.lidar_end_time = m_package.lidar_start_time + m_package.cloud_in->points.back().curvature;
        m_state.lidar_pushed = true;
    }

    if (m_state.last_imu_time < m_package.lidar_end_time)
        return false;

    std::deque<IMUDate>().swap(m_package.imu_data_in);
    {
        boost::lock_guard<boost::mutex> lock(m_state.imu_mutex);
        while (!m_state.imu_queue.empty() && m_state.imu_queue.front().time < m_package.lidar_end_time)
        {
            m_package.imu_data_in.push_back(m_state.imu_queue.front());
            m_state.imu_queue.pop_front();
        }
    }
    {
        boost::lock_guard<boost::mutex> lock(m_state.lidar_mutex);
        m_state.lidar_queue.pop_front();
    }
    m_state.lidar_pushed = false;

    if (m_package.imu_data_in.front().time < m_package.first_point_time)
    {
        boost::lock_guard<boost::mutex> lock(m_state.imu_mutex);
        while (!m_package.imu_data_in.empty() && m_package.imu_data_in.front().time < m_package.first_point_time)
            m_package.imu_data_in.pop_front();
    }
    if (m_package.imu_data_in.empty())
        return false;
    ROS_DEBUG("imu size: %lu, cloud_size: %lu", m_package.imu_data_in.size(), m_package.cloud_in->points.size());

    return true;
}

void PointLIOROS::broadcastTf()
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time().fromSec(m_package.lidar_end_time);
    transform.header.frame_id = m_config.map_frame;
    transform.child_frame_id = m_config.body_frame;
    transform.transform.translation.x = m_map_builder->eskf.x.p.x();
    transform.transform.translation.y = m_map_builder->eskf.x.p.y();
    transform.transform.translation.z = m_map_builder->eskf.x.p.z();
    transform.transform.rotation.x = m_map_builder->eskf.x.q.x();
    transform.transform.rotation.y = m_map_builder->eskf.x.q.y();
    transform.transform.rotation.z = m_map_builder->eskf.x.q.z();
    transform.transform.rotation.w = m_map_builder->eskf.x.q.w();
    m_tf_broadcaster.sendTransform(transform);
}

void PointLIOROS::publishOdom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(m_package.lidar_end_time);
    odom.header.frame_id = m_config.map_frame;
    odom.child_frame_id = m_config.body_frame;
    odom.pose.pose.position.x = m_map_builder->eskf.x.p.x();
    odom.pose.pose.position.y = m_map_builder->eskf.x.p.y();
    odom.pose.pose.position.z = m_map_builder->eskf.x.p.z();
    odom.pose.pose.orientation.x = m_map_builder->eskf.x.q.x();
    odom.pose.pose.orientation.y = m_map_builder->eskf.x.q.y();
    odom.pose.pose.orientation.z = m_map_builder->eskf.x.q.z();
    odom.pose.pose.orientation.w = m_map_builder->eskf.x.q.w();
    Vec3d vec = m_map_builder->eskf.x.q.inverse() * m_map_builder->eskf.x.v;
    odom.twist.twist.linear.x = vec.x();
    odom.twist.twist.linear.y = vec.y();
    odom.twist.twist.linear.z = vec.z();
    odom.twist.twist.angular.x = m_map_builder->eskf.x.omega.x();
    odom.twist.twist.angular.y = m_map_builder->eskf.x.omega.y();
    odom.twist.twist.angular.z = m_map_builder->eskf.x.omega.z();
    m_odom_pub.publish(odom);
}

PointLIOROS::~PointLIOROS()
{
    m_state.is_activate = false;
    m_main_thread.join();
}