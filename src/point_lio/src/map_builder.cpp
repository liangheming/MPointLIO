#include "map_builder.h"

MapBuilder::MapBuilder(const MapBuilderConfig &config)
    : m_config(config)
{
    reset();
    eskf.lidar_measure_func = std::bind(&MapBuilder::lidarMeasure, this, std::placeholders::_1, std::placeholders::_2);
    eskf.imu_measure_func = std::bind(&MapBuilder::imuMeasure, this, std::placeholders::_1, std::placeholders::_2);
}

void MapBuilder::lidarToBody(CloudType::Ptr &in, CloudType::Ptr &out)
{
    pcl::transformPointCloud(*in, *out, eskf.x.offset_p, eskf.x.offset_q);
}

void MapBuilder::lidarToWorld(CloudType::Ptr &in, CloudType::Ptr &out)
{
    Vec3d p = eskf.x.p + eskf.x.q * eskf.x.offset_p;
    Quatd q = eskf.x.q * eskf.x.offset_q;
    pcl::transformPointCloud(*in, *out, p, q);
}

void MapBuilder::bodyToWorld(CloudType::Ptr &in, CloudType::Ptr &out)
{
    pcl::transformPointCloud(*in, *out, eskf.x.p, eskf.x.q);
}

void MapBuilder::reset()
{
    m_status = Status::IMU_INIT;
    m_cache_imu_data.clear();
    m_cache_imu_data.reserve(1000);
    m_cache_clouds.reset(new CloudType);
    m_cache_clouds->clear();
    m_cache_clouds->reserve(100000);
    m_ikd_tree.reset(new KD_TREE<PointType>());
    m_ikd_tree->set_downsample_param(m_config.map_resolution);
    m_last_process_time = -1.0;
    m_last_map_update_time = -1.0;

    m_local_map.cub_to_rm.clear();
    m_local_map.initialed = false;
}

bool MapBuilder::initIMU()
{
    if (m_cache_imu_data.size() < m_config.imu_init_num)
        return false;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();
    for (int i = 0; i < m_config.imu_init_num; i++)
    {
        acc += m_cache_imu_data[i].acc;
        gyro += m_cache_imu_data[i].gyr;
    }
    acc /= m_config.imu_init_num;
    gyro /= m_config.imu_init_num;
    eskf.x.bg = gyro;
    Quatd init_q = Quatd::FromTwoVectors(-acc.normalized(), Vec3d(0, 0, -1));
    eskf.x.q = init_q;
    eskf.x.g = Vec3d(0, 0, -GRAVITY);
    eskf.x.acc = init_q.inverse() * eskf.x.g * (-1.0);
    eskf.x.offset_q = m_config.q_il;
    eskf.x.offset_p = m_config.t_il;
    eskf.P = Mat30d::Identity() * 0.1;
    eskf.P.block<3, 3>(State::V_ID, State::V_ID) = Mat3d::Identity() * m_config.vel_cov;
    eskf.P.block<3, 3>(State::ACC_ID, State::ACC_ID) = Mat3d::Identity() * m_config.acc_cov;
    eskf.P.block<3, 3>(State::OMEGA_ID, State::OMEGA_ID) = Mat3d::Identity() * m_config.gyro_cov;
    eskf.P.block<3, 3>(State::BA_ID, State::BA_ID) = Mat3d::Identity() * m_config.b_acc_cov;
    eskf.P.block<3, 3>(State::BG_ID, State::BG_ID) = Mat3d::Identity() * m_config.b_gyro_cov;

    eskf.Q.setZero();
    eskf.Q.block<3, 3>(0, 0) = Mat3d::Identity() * m_config.b_gyro_cov;
    eskf.Q.block<3, 3>(3, 3) = Mat3d::Identity() * m_config.b_acc_cov;
    eskf.Q.block<3, 3>(6, 6) = Mat3d::Identity() * m_config.gyro_cov;
    eskf.Q.block<3, 3>(9, 9) = Mat3d::Identity() * m_config.acc_cov;

    return true;
}

bool MapBuilder::initMap()
{
    CloudType::Ptr cloud(new CloudType);
    lidarToWorld(m_cache_clouds, cloud);
    m_ikd_tree->Build(cloud->points);
    return true;
}

void MapBuilder::lidarMeasure(State &x, ObservationState &obs)
{
    PointVec points_near;
    std::vector<float> point_sq_dist(m_config.near_search_num);
    m_ikd_tree->Nearest_Search(m_current_world_point, m_config.near_search_num, points_near, point_sq_dist);
    m_current_world_point.curvature = float(points_near.size());
    if (points_near.size() < m_config.near_search_num || point_sq_dist[m_config.near_search_num - 1] > 4.0)
    {
        obs.valid = false;
        return;
    }
    Vec4d pabcd;
    if (!esti_plane(points_near, 0.1, pabcd))
    {
        obs.valid = false;
        return;
    }

    m_current_world_point.normal_x = points_near[0].x;
    m_current_world_point.normal_y = points_near[0].y;
    m_current_world_point.normal_z = points_near[0].z;

    Vec3d normal = pabcd.head<3>();
    double d = normal.dot(m_current_point_in_world) + pabcd(3);
    Eigen::Matrix<double, 1, 6> drdx;
    drdx.block<1, 3>(0, 0) = -normal.transpose() * eskf.x.q.matrix() * skew(m_current_point_in_body);
    drdx.block<1, 3>(0, 3) = normal;
    obs.H.block<6, 6>(0, 0) = drdx.transpose() * m_config.lidar_meas_cov_inv * drdx;
    obs.z.head<6>() = drdx.transpose() * m_config.lidar_meas_cov_inv * d;
    obs.valid = true;
}

void MapBuilder::imuMeasure(State &x, ObservationState &obs)
{
    Eigen::Matrix<double, 6, 1> res;
    res.segment<3>(0) = x.omega + x.bg - m_current_imu_gyro_obs;
    res.segment<3>(3) = x.acc + x.ba - m_current_imu_acc_obs;
    Eigen::Matrix<double, 6, 30> drdx = Eigen::Matrix<double, 6, 30>::Zero();
    drdx.block<3, 3>(0, State::BG_ID) = Mat3d::Identity();
    drdx.block<3, 3>(0, State::OMEGA_ID) = Mat3d::Identity();
    drdx.block<3, 3>(3, State::BA_ID) = Mat3d::Identity();
    drdx.block<3, 3>(3, State::ACC_ID) = Mat3d::Identity();
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
    R.block<3, 3>(0, 0) *= m_config.imu_meas_gyro_cov_inv;
    R.block<3, 3>(3, 3) *= m_config.imu_meas_acc_cov_inv;
    obs.H = drdx.transpose() * R * drdx;
    obs.z = drdx.transpose() * R * res;
    obs.valid = true;
}

bool MapBuilder::mapping(Package &package)
{
    std::deque<IMUDate>::iterator imu_iter = package.imu_data_in.begin();
    PointVec::iterator cloud_iter = package.cloud_in->points.begin();
    while (cloud_iter != package.cloud_in->points.end())
    {
        double point_time = package.lidar_start_time + cloud_iter->curvature;
        if (point_time < imu_iter->time || imu_iter == package.imu_data_in.end())
        {
            double dt = point_time - m_last_process_time;
            if (dt < 1e-6)
            {
                cloud_iter++;
                continue;
            }
            eskf.predict(dt);
            m_current_point_in_lidar = Vec3d(cloud_iter->x, cloud_iter->y, cloud_iter->z);
            m_current_point_in_body = eskf.x.offset_q * m_current_point_in_lidar + eskf.x.offset_p;
            m_current_point_in_world = eskf.x.q * m_current_point_in_body + eskf.x.p;
            m_current_world_point.x = m_current_point_in_world(0);
            m_current_world_point.y = m_current_point_in_world(1);
            m_current_world_point.z = m_current_point_in_world(2);
            m_current_world_point.intensity = cloud_iter->intensity;
            eskf.updateByLidar();
            m_current_point_in_world = eskf.x.q * m_current_point_in_body + eskf.x.p;
            m_current_world_point.x = m_current_point_in_world(0);
            m_current_world_point.y = m_current_point_in_world(1);
            m_current_world_point.z = m_current_point_in_world(2);

            m_cache_clouds->points.push_back(m_current_world_point);

            m_last_process_time = point_time;
            cloud_iter++;
        }
        else
        {
            double dt = imu_iter->time - m_last_process_time;
            eskf.predict(dt);
            m_current_imu_acc_obs = imu_iter->acc;
            m_current_imu_gyro_obs = imu_iter->gyr;
            eskf.updateByIMU();
            m_last_process_time = imu_iter->time;
            imu_iter++;
        }
    }

    return true;
}

void MapBuilder::process(Package &package)
{
    if (m_status == Status::IMU_INIT)
    {
        m_cache_clouds->points.insert(m_cache_clouds->points.end(), package.cloud_in->points.begin(), package.cloud_in->points.end());
        m_cache_imu_data.insert(m_cache_imu_data.end(), package.imu_data_in.begin(), package.imu_data_in.end());
        if (initIMU())
            m_status = Status::MAP_INIT;
    }
    else if (m_status == Status::MAP_INIT)
    {
        initMap();
        m_last_process_time = package.lidar_end_time;
        m_last_map_update_time = m_last_process_time;
        m_cache_clouds->clear();
        m_cache_imu_data.clear();
        m_status = Status::MAPPING;
    }
    else
    {
        mapping(package);
    }
}

void MapBuilder::trimMap()
{
    m_local_map.cub_to_rm.clear();
    // Vec3d pos_lidar = eskf.x.q * eskf.x.offset_p + eskf.x.p;
    Vec3d pos_lidar = eskf.x.p;
    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;

    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikd_tree->acquire_removed_points(points_history);
    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikd_tree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

void MapBuilder::increMap()
{
    if (m_cache_clouds->points.size() == 0)
        return;

    int size = m_cache_clouds->size();

    PointVec point_to_add;
    PointVec point_no_need_downsample;

    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cache_clouds->points[i];
        if (p.curvature < m_config.near_search_num)
        {
            point_to_add.push_back(p);
            continue;
        }

        PointType mid_point;
        mid_point.x = std::floor(p.x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(p.y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(p.z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        if (fabs(p.normal_x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(p.normal_y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(p.normal_z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(p);
            continue;
        }
        float dist = (p.x - mid_point.x) * (p.x - mid_point.x) + (p.y - mid_point.y) * (p.y - mid_point.y) + (p.z - mid_point.z) * (p.z - mid_point.z);
        float dist_for_nearest = (p.normal_x - mid_point.x) * (p.normal_x - mid_point.x) + (p.normal_y - mid_point.y) * (p.normal_y - mid_point.y) + (p.normal_z - mid_point.z) * (p.normal_z - mid_point.z);
        if (dist_for_nearest > dist)
            point_to_add.push_back(p);
    }
    m_ikd_tree->Add_Points(point_to_add, true);
    m_ikd_tree->Add_Points(point_no_need_downsample, false);
    // 清理缓存
    m_last_map_update_time = m_last_process_time;
    m_cache_clouds->clear();
}