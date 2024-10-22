#include "commons.h"

CloudType::Ptr livox2PCL(const livox_ros_driver2::CustomMsg::ConstPtr &msg, int filter_num, double min_range, double max_range, double time_thresh, double downsample)
{
    CloudType::Ptr cloud(new CloudType);
    int point_num = msg->point_num;
    cloud->points.reserve(point_num / filter_num + 1);
    double range_min_sqrd = min_range * min_range;
    double range_max_sqrd = max_range * max_range;
    double start_time = msg->header.stamp.toSec();

    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            PointType p;
            p.x = msg->points[i].x;
            p.y = msg->points[i].y;
            p.z = msg->points[i].z;
            p.intensity = msg->points[i].reflectivity;
            double offset = msg->points[i].offset_time / 1000000000.0;
            if (start_time + offset < time_thresh)
                continue;
            p.curvature = static_cast<float>(offset);
            if (p.x * p.x + p.y * p.y + p.z * p.z < range_min_sqrd || p.x * p.x + p.y * p.y + p.z * p.z > range_max_sqrd)
                continue;
            cloud->points.push_back(p);
        }
    }
    if (downsample > 0.0)
    {
        pcl::VoxelGrid<PointType> filter;
        filter.setLeafSize(downsample, downsample, downsample);
        filter.setInputCloud(cloud);
        filter.filter(*cloud);
    }

    std::sort(cloud->points.begin(), cloud->points.end(), [](const PointType &a, const PointType &b)
              { return a.curvature < b.curvature; });
    return cloud;
}

bool esti_plane(PointVec &points, const double &thresh, Vec4d &out)
{
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::MatrixXd b(points.size(), 1);
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (size_t i = 0; i < points.size(); i++)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }
    Vec3d normvec = A.colPivHouseholderQr().solve(b);
    double norm = normvec.norm();
    out[0] = normvec(0) / norm;
    out[1] = normvec(1) / norm;
    out[2] = normvec(2) / norm;
    out[3] = 1.0 / norm;
    for (size_t j = 0; j < points.size(); j++)
    {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
            return false;
    }
    return true;
}
Mat3d skew(const Vec3d &x)
{
    Mat3d res;
    res << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
    return res;
}
Mat3d leftJacobianInverse(const Vec3d &x)
{
    double theta = x.norm();
    double theta_sq = theta * theta;
    Mat3d Omega = skew(x);
    Mat3d V_inv;
    if (theta < 1e-6)
        V_inv = Mat3d::Identity() - 0.5 * Omega + (1. / 12.) * (Omega * Omega);
    else
        V_inv = Mat3d::Identity() - 0.5 * Omega + (1.0 - 0.5 * theta * cos(0.5 * theta) / sin(0.5 * theta)) / (theta_sq) * (Omega * Omega);
    return V_inv;
}

Mat3d Exp(const Vec3d &x)
{
    double theta = x.norm();
    if (theta < 1e-6)
        return Mat3d::Identity();
    else
    {
        Vec3d omega = x.normalized();
        Mat3d skew_omega = skew(omega);
        double cos_theta = cos(theta);
        return cos_theta * Mat3d::Identity() + (1.0 - cos_theta) * omega * omega.transpose() + sin(theta) * skew_omega;
    }
}
