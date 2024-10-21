#pragma once
#include <queue>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livox_ros_driver2/CustomMsg.h>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec3f = Eigen::Vector3f;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Quatd = Eigen::Quaterniond;
using Vec30d = Eigen::Matrix<double, 30, 1>;
using Mat30d = Eigen::Matrix<double, 30, 30>;
using Mat12d = Eigen::Matrix<double, 12, 12>;
using Mat30x12d = Eigen::Matrix<double, 30, 12>;

constexpr double GRAVITY = 9.8178;

struct IMUDate
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time;
    Vec3d acc;
    Vec3d gyr;
    IMUDate(double t, double a1, double a2, double a3, double g1, double g2, double g3) : time(t), acc(a1, a2, a3), gyr(g1, g2, g3) {}
};

struct Package
{
    double lidar_start_time;
    double first_point_time;
    double lidar_end_time;
    CloudType::Ptr cloud_in;
    CloudType::Ptr cloud_body;
    CloudType::Ptr cloud_world;
    std::deque<IMUDate> imu_data_in;
    Package() : cloud_in(new CloudType), cloud_body(new CloudType), cloud_world(new CloudType) {}
};

CloudType::Ptr livox2PCL(const livox_ros_driver2::CustomMsg::ConstPtr &msg, int filter_num, double min_range, double max_range, double time_thresh = 0.0);

bool esti_plane(PointVec &points, const double &thresh, Vec4d &out);
