#pragma once
#include "commons.h"
#include "sophus/se3.hpp"

struct State
{
    Quatd q;
    Vec3d p;
    Quatd offset_q;
    Vec3d offset_p;
    Vec3d v;
    Vec3d bg;
    Vec3d ba;
    Vec3d g;
    Vec3d omega;
    Vec3d acc;
    State() : p(0, 0, 0), v(0, 0, 0), q(1, 0, 0, 0), bg(0, 0, 0), ba(0, 0, 0), g(0, 0, -GRAVITY), omega(0, 0, 0), acc(0, 0, 0), offset_p(0, 0, 0), offset_q(1, 0, 0, 0) {}
    void operator+=(const Vec30d &delta);
    static size_t Q_ID, P_ID, OFF_Q_ID, OFF_P_ID, V_ID, BG_ID, BA_ID, G_ID, OMEGA_ID, ACC_ID;
    void print();
};

struct ObservationState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mat30d H;
    Vec30d z;
    bool valid;
    ObservationState() : H(Mat30d::Zero()), z(Vec30d::Zero()), valid(false) {}
};
using MeasureFunc = std::function<void(State &, ObservationState &)>;

class ESKF
{
public:
    State x;
    Mat30d P;
    Mat12d Q;
    MeasureFunc imu_measure_func;
    MeasureFunc lidar_measure_func;
    ESKF() : x() {}
    void predict(double dt);
    bool updateByIMU();
    bool updateByLidar();
};