#include "eskf.h"

size_t State::Q_ID = 0;
size_t State::P_ID = 3;
size_t State::OFF_Q_ID = 6;
size_t State::OFF_P_ID = 9;
size_t State::V_ID = 12;
size_t State::BG_ID = 15;
size_t State::BA_ID = 18;
size_t State::G_ID = 21;
size_t State::OMEGA_ID = 24;
size_t State::ACC_ID = 27;

void State::operator+=(const Vec30d &delta)
{
    Vec3d delta_q = delta.segment<3>(Q_ID);
    q = (q * Quatd(1.0, delta_q(0) / 2, delta_q(1) / 2, delta_q(2) / 2)).normalized();
    p += delta.segment<3>(P_ID);
    Vec3d delta_offset_q = delta.segment<3>(OFF_Q_ID);
    offset_q = (offset_q * Quatd(1.0, delta_offset_q(0) / 2, delta_offset_q(1) / 2, delta_offset_q(2) / 2)).normalized();
    offset_p += delta.segment<3>(OFF_P_ID);
    v += delta.segment<3>(V_ID);
    bg += delta.segment<3>(BG_ID);
    ba += delta.segment<3>(BA_ID);
    g += delta.segment<3>(G_ID);
    omega += delta.segment<3>(OMEGA_ID);
    acc += delta.segment<3>(ACC_ID);
}

void State::print()
{
    std::cout << "=================state=================" << std::endl;
    std::cout << "q:" << q.coeffs().transpose() << std::endl;
    std::cout << "p:" << p.transpose() << std::endl;
    std::cout << "offset_q:" << offset_q.coeffs().transpose() << std::endl;
    std::cout << "offset_p:" << offset_p.transpose() << std::endl;
    std::cout << "v:" << v.transpose() << std::endl;
    std::cout << "bg:" << bg.transpose() << std::endl;
    std::cout << "ba:" << ba.transpose() << std::endl;
    std::cout << "g:" << g.transpose() << std::endl;
    std::cout << "omega:" << omega.transpose() << std::endl;
    std::cout << "acc:" << acc.transpose() << std::endl;
}

void ESKF::predict(double dt)
{
    Vec30d delta = Vec30d::Zero();
    delta.segment<3>(State::Q_ID) = x.omega * dt;
    delta.segment<3>(State::P_ID) = x.v * dt;
    delta.segment<3>(State::V_ID) = (x.q * x.acc + x.g) * dt;
    x += delta;
    Mat30d Fx = Mat30d::Identity();
    Mat30x12d Fg = Mat30x12d::Zero();
    Fx.block<3, 3>(State::Q_ID, State::Q_ID) = Sophus::SO3d::exp(-x.omega * dt).matrix();
    Fx.block<3, 3>(State::Q_ID, State::OMEGA_ID) = Mat3d::Identity() * dt;
    Fx.block<3, 3>(State::P_ID, State::V_ID) = Mat3d::Identity() * dt;
    Fx.block<3, 3>(State::V_ID, State::Q_ID) = -x.q.matrix() * Sophus::SO3d::hat(x.acc) * dt;
    Fx.block<3, 3>(State::V_ID, State::G_ID) = Mat3d::Identity() * dt;
    Fx.block<3, 3>(State::V_ID, State::ACC_ID) = x.q.matrix() * dt;

    Fg.block<3, 3>(State::BG_ID, 0) = Mat3d::Identity() * dt;
    Fg.block<3, 3>(State::BA_ID, 3) = Mat3d::Identity() * dt;
    Fg.block<3, 3>(State::OMEGA_ID, 6) = Mat3d::Identity() * dt;
    Fg.block<3, 3>(State::ACC_ID, 9) = Mat3d::Identity() * dt;
    P = Fx * P * Fx.transpose() + Fg * Q * Fg.transpose();
}

bool ESKF::updateByIMU()
{
    ObservationState obs;
    imu_measure_func(x, obs);
    if (!obs.valid)
        return false;
    Mat30d H = P.inverse();
    H += obs.H;
    Mat30d H_inv = H.inverse();
    Vec30d delta = -H_inv * obs.z;
    x += delta;
    Mat30d K = Mat30d::Identity();
    K.block<3, 3>(State::Q_ID, State::Q_ID) = Sophus::SO3d::leftJacobianInverse(delta.segment<3>(State::Q_ID)).transpose();
    K.block<3, 3>(State::OFF_Q_ID, State::OFF_Q_ID) = Sophus::SO3d::leftJacobianInverse(delta.segment<3>(State::OFF_Q_ID)).transpose();
    P = K * H_inv * K.transpose();
    return true;
}

bool ESKF::updateByLidar()
{
    ObservationState obs;
    lidar_measure_func(x, obs);
    if (!obs.valid)
        return false;
    Mat30d H = P.inverse();
    H += obs.H;
    Mat30d H_inv = H.inverse();
    Vec30d delta = -H_inv * obs.z;
    x += delta;
    Mat30d K = Mat30d::Identity();
    K.block<3, 3>(State::Q_ID, State::Q_ID) = Sophus::SO3d::leftJacobianInverse(delta.segment<3>(State::Q_ID)).transpose();
    K.block<3, 3>(State::OFF_Q_ID, State::OFF_Q_ID) = Sophus::SO3d::leftJacobianInverse(delta.segment<3>(State::OFF_Q_ID)).transpose();
    P = K * H_inv * K.transpose();
    return true;
}