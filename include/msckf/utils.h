#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace MSCKF {

inline Eigen::Matrix<double, 21, 21> Msckf::getMatrixExponential(const Eigen::Matrix3d F, double dt) {
    Matrix<double, 21, 21> Fdt = F * dtime;
    Matrix<double, 21, 21> Fdt_sqaure = Fdt * F;
    Matrix<double, 21, 21> Fdt_cube = Fdt_sqaure * F;

    return Matrix<double, 21, 21>::Identity() + Fdt + 0.5 * Fdt_square + (1.0 / 6.0) * Fdt_cube;
}

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d cross;
    cross << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return cross;
}

inline Eigen::Matrix3d quaternionToRotation(const Eigen::Vector4d &q) {
    Eigen::Quaterniond q_(q[3], q[0], q[1], q[2]);
    Eigen::Matrix3d R = q_.normalized().toRotationMatrix();
    return R;
}

inline Eigen::Vector4d rotationToQuaternion(const Eigen::Matrix3d &R) {
    Eigen::Quaterniond q(R);
    Eigen::Vector4d orientation(q.x(), q.y(), q.z(), q.w());
    return orientation;
}

inline Eigen::Matrix4d findOmega(const Eigen::Vector3d &w) {
    Eigen::Matrix4d Omega;
    Omega.block<3, 3>(0, 0) = -skewMatrix(w);
    Omega.block<3, 1>(0, 3) = w;
    Omega.block<1, 3>(3, 0) = -w.transpose();
    Omega(3, 3) = 0.0;
    return Omega;
}

}  // namespace MSCKF

#endif