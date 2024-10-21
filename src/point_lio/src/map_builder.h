#pragma once
#include "commons.h"
#include "eskf.h"
#include "ikd_Tree.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

struct MapBuilderConfig
{
    int imu_init_num = 100;
    double vel_cov = 20.0;
    double acc_cov = 500;
    double gyro_cov = 1000;
    double b_acc_cov = 0.0001;
    double b_gyro_cov = 0.0001;
    double lidar_meas_cov_inv = 100;
    double imu_meas_acc_cov_inv = 10;
    double imu_meas_gyro_cov_inv = 10;
    double satu_acc = 20.0;
    double satu_gyro = 35;
    double scan_resolution = 0.1;
    double map_resolution = 0.3;
    double near_search_num = 5;

    double cube_len = 300.0;
    double move_thresh = 1.5;
    double det_range = 60.0;
};

struct LocalMap
{
    bool initialed;
    BoxPointType local_map_corner;
    std::vector<BoxPointType> cub_to_rm;
};

class MapBuilder
{
public:
    enum class Status
    {
        IMU_INIT,
        MAP_INIT,
        MAPPING,
    };
    MapBuilder(const MapBuilderConfig &config);
    void process(Package &package);
    bool initIMU();
    bool initMap();
    bool mapping(Package &package);

    void reset();

    void lidarToBody(CloudType::Ptr &in, CloudType::Ptr &out);

    void lidarToWorld(CloudType::Ptr &in, CloudType::Ptr &out);

    void bodyToWorld(CloudType::Ptr &in, CloudType::Ptr &out);

    void lidarMeasure(State &x, ObservationState &obs);

    void imuMeasure(State &x, ObservationState &obs);

    void trimMap();

    void increMap();

    double lastMapUpdateTime() { return m_last_map_update_time; }

    Status status() { return m_status; }

    CloudType::Ptr cacheClouds() { return m_cache_clouds; }

    ESKF eskf;

private:
    Status m_status;
    MapBuilderConfig m_config;
    double m_last_map_update_time;
    double m_last_process_time;
    std::vector<IMUDate> m_cache_imu_data;
    CloudType::Ptr m_cache_clouds;
    std::shared_ptr<KD_TREE<PointType>> m_ikd_tree;

    Vec3d m_current_point_in_lidar;
    Vec3d m_current_point_in_body;
    Vec3d m_current_point_in_world;
    PointType m_current_world_point;
    Vec3d m_current_imu_acc_obs;
    Vec3d m_current_imu_gyro_obs;

    LocalMap m_local_map;
};