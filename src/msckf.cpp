#include "msckf/msckf.h"

namespace MSCKF {

// RK5 integration step using Dormand-Prince method
void MSCKF::rk5Step(const double dt, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc) {
    // Sample the noises
    imu_state.sampleNoise();
    IMU::IMUStateVariable X = imu_state.state;

    printf("Starting RK5 step with dt=%.6f \n", dt);

    // Calculate gyro norm
    double gyro_norm = gyro.norm();

    // Create the skew-symmetric matrix Omega for quaternion derivative
    Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
    Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
    Omega.block<3, 1>(0, 3) = gyro;
    Omega.block<1, 3>(3, 0) = -gyro;

    // Current state
    Eigen::Vector4d &q = imu_state.orientation;
    Eigen::Vector3d &v = imu_state.velocity;
    Eigen::Vector3d &p = imu_state.position;

    // Pre-compute quaternion derivatives for different time steps
    Eigen::Vector4d dq_dt, dq_dt5, dq_dt3, dq_dt8, dq_dt9;

    if (gyro_norm > 1e-5) {
        // Full step (dt)
        dq_dt = (cos(gyro_norm * 0.5 * dt) * Eigen::Matrix4d::Identity() +
                 (1.0 / gyro_norm) * sin(gyro_norm * 0.5 * dt) * Omega) *
                q;

        // 1/5 step
        dq_dt5 = (cos(gyro_norm * 0.1 * dt) * Eigen::Matrix4d::Identity() +
                  (1.0 / gyro_norm) * sin(gyro_norm * 0.1 * dt) * Omega) *
                 q;

        // 3/10 step
        dq_dt3 = (cos(gyro_norm * 0.15 * dt) * Eigen::Matrix4d::Identity() +
                  (1.0 / gyro_norm) * sin(gyro_norm * 0.15 * dt) * Omega) *
                 q;

        // 4/5 step
        dq_dt8 = (cos(gyro_norm * 0.4 * dt) * Eigen::Matrix4d::Identity() +
                  (1.0 / gyro_norm) * sin(gyro_norm * 0.4 * dt) * Omega) *
                 q;

        // 8/9 step
        dq_dt9 = (cos(gyro_norm * 0.44444 * dt) * Eigen::Matrix4d::Identity() +
                  (1.0 / gyro_norm) * sin(gyro_norm * 0.44444 * dt) * Omega) *
                 q;
    } else {
        // Small angle approximation for numerical stability
        dq_dt = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega) * q;
        dq_dt5 = (Eigen::Matrix4d::Identity() + 0.1 * dt * Omega) * q;
        dq_dt3 = (Eigen::Matrix4d::Identity() + 0.15 * dt * Omega) * q;
        dq_dt8 = (Eigen::Matrix4d::Identity() + 0.4 * dt * Omega) * q;
        dq_dt9 = (Eigen::Matrix4d::Identity() + 0.44444 * dt * Omega) * q;
    }

    // Rotation matrices for each stage
    Eigen::Matrix3d R_current = quaternionToRotation(q);
    Eigen::Matrix3d R_dt5 = quaternionToRotation(dq_dt5);
    Eigen::Matrix3d R_dt3 = quaternionToRotation(dq_dt3);
    Eigen::Matrix3d R_dt8 = quaternionToRotation(dq_dt8);
    Eigen::Matrix3d R_dt9 = quaternionToRotation(dq_dt9);
    Eigen::Matrix3d R_dt = quaternionToRotation(dq_dt);

    // Dormand-Prince integration steps for velocity and position

    // k1 = f(tn, yn)
    Eigen::Vector3d k1_p_dot = v;
    Eigen::Vector3d k1_v_dot = R_current * acc - g;

    // k2 = f(tn + 1/5*dt, yn + dt*(1/5*k1))
    Eigen::Vector3d k1_v = v + k1_v_dot * dt * (1.0 / 5.0);
    Eigen::Vector3d k2_v_dot = R_dt5 * acc - g;
    Eigen::Vector3d k2_p_dot = k1_v;

    // k3 = f(tn + 3/10*dt, yn + dt*(3/40*k1 + 9/40*k2))
    Eigen::Vector3d k2_v = v + dt * (3.0 / 40.0 * k1_v_dot + 9.0 / 40.0 * k2_v_dot);
    Eigen::Vector3d k3_v_dot = R_dt3 * acc - g;
    Eigen::Vector3d k3_p_dot = k2_v;

    // k4 = f(tn + 4/5*dt, yn + dt*(44/45*k1 - 56/15*k2 + 32/9*k3))
    Eigen::Vector3d k3_v = v + dt * (44.0 / 45.0 * k1_v_dot - 56.0 / 15.0 * k2_v_dot + 32.0 / 9.0 * k3_v_dot);
    Eigen::Vector3d k4_v_dot = R_dt8 * acc - g;
    Eigen::Vector3d k4_p_dot = k3_v;

    // k5 = f(tn + 8/9*dt, yn + dt*(19372/6561*k1 - 25360/2187*k2 + 64448/6561*k3 - 212/729*k4))
    Eigen::Vector3d k4_v = v + dt * (19372.0 / 6561.0 * k1_v_dot - 25360.0 / 2187.0 * k2_v_dot +
                                     64448.0 / 6561.0 * k3_v_dot - 212.0 / 729.0 * k4_v_dot);
    Eigen::Vector3d k5_v_dot = R_dt9 * acc - g;
    Eigen::Vector3d k5_p_dot = k4_v;

    // k6 = f(tn + dt, yn + dt*(9017/3168*k1 - 355/33*k2 + 46732/5247*k3 + 49/176*k4 - 5103/18656*k5))
    Eigen::Vector3d k5_v = v + dt * (9017.0 / 3168.0 * k1_v_dot - 355.0 / 33.0 * k2_v_dot +
                                     46732.0 / 5247.0 * k3_v_dot + 49.0 / 176.0 * k4_v_dot -
                                     5103.0 / 18656.0 * k5_v_dot);
    Eigen::Vector3d k6_v_dot = R_dt * acc - g;
    Eigen::Vector3d k6_p_dot = k5_v;

    // k7 = f(tn + dt, yn + dt*(35/384*k1 + 500/1113*k3 + 125/192*k4 - 2187/6784*k5 + 11/84*k6))
    Eigen::Vector3d k6_v = v + dt * (35.0 / 384.0 * k1_v_dot + 500.0 / 1113.0 * k3_v_dot +
                                     125.0 / 192.0 * k4_v_dot - 2187.0 / 6784.0 * k5_v_dot +
                                     11.0 / 84.0 * k6_v_dot);
    Eigen::Vector3d k7_v_dot = R_dt * acc - g;
    Eigen::Vector3d k7_p_dot = k6_v;

    // Update quaternion and normalize it
    q = dq_dt;
    q.normalize();

    // Update velocity and position using Dormand-Prince 5th order formula
    v = v + dt * (35.0 / 384.0 * k1_v_dot + 500.0 / 1113.0 * k3_v_dot +
                  125.0 / 192.0 * k4_v_dot - 2187.0 / 6784.0 * k5_v_dot +
                  11.0 / 84.0 * k6_v_dot);

    p = p + dt * (35.0 / 384.0 * k1_p_dot + 500.0 / 1113.0 * k3_p_dot +
                  125.0 / 192.0 * k4_p_dot - 2187.0 / 6784.0 * k5_p_dot +
                  11.0 / 84.0 * k6_p_dot);
    return;
}

void MSCKF::computeJacobians(Eigen::Matrix3d &I_R_G, Eigen::Vector3d &gyro, Eigen::Vector3d &acc) {
    // Fill in the F matrix (state transition matrix) blocks
    F.block<3, 3>(0, 0) = -skewSymmetric(gyro);
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
    F.block<3, 3>(6, 0) = -I_R_G.transpose() * skewSymmetric(acc);
    F.block<3, 3>(6, 9) = -I_R_G.transpose();
    F.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();

    G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(6, 6) = -I_R_G.transpose();
    G.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
}

void MSCKF::propagateIMU(double dt, const Eigen::Vector3d &gyro_m, const Eigen::Vector3d &acc_m) {
    // Rotation matrix from IMU to Global frame
    Eigen::Matrix3d I_R_G = quaternionToRotation(imu_state.orientation);

    Eigen::Vector3d gyro = gyro_m - imu_state.gyro_bias;
    Eigen::Vector3d acc = acc_m - imu_state.acc_bias;

    computeJacobians(I_R_G, gyro, acc);

    // Caluclate the matrix expoential
    Eigen::Matrix<double, 21, 21> Phi = getMatrixExponential(F, dt);
    rk5Step(dt, gyro, acc);

    applyConstraints(dt, I_R_G, Phi);
    propagateCovariance(dt, Phi);
    stateAugmentation();
}

// https://seanbow.com/papers/tr_2012_1.pdf. Check Section 4.1.1
void MSCKF::applyConstraints(const double dt, const Eigen::Matrix3d &I_R_G, const Eigen::Matrix<double, 21, 21> &Phi) {
    // Get previous rotation
    C_prev = quaternionToRotation(imu_state.orientation_null);
    // Get relative rotation between curreFnt and previous frame
    Phi.block<3, 3>(0, 0) = I_R_G * C_prev.transpose();

    Eigen::Vector3d u = C_prev * g;

    // Get velocity constraints
    Eigen::Matrix3d A_31 = Phi.block<3, 3>(6, 0);
    Eigen::Vector3d w_31 = skewSymmetric(imu_state.velocity_null) * g - skewSymmetric(imu_state.velocity) * g;
    Phi.block<3, 3>(6, 0) = A_31 - (A_31 * u - w_31) * (u.transpose() * u).inverse() * u.transpose();

    // Get position constraints
    Eigen::Matrix3d A_51 = Phi.block<3, 3>(12, 0);
    Eigen::Vector3d w_51 = dt * skewSymmetric(imu_state.velocity_null) * g - skewSymmetric(imu_state.velocity) * g;
    Phi.block<3, 3>(12, 0) = A_51 - (A_51 * u - w_51) * (u.transpose() * u).inverse() * u.transpose();
}

// Propagates the state error covariance matrix
void MSCKF::propagateCovariance(const double dt, const Eigen::Matrix<double, 21, 21> &Phi) {
    // Calculate discrete noise covaraince
    Eigen::Matrix<double, 15, 15> Q = Phi * G * state_cov * G.transpose() * Phi.transpose() * dt;
    // Propagate the covaraince
    state_cov.block<15, 15>(0, 0) = Phi * state_cov * Phi.transpose() + Q;
    // Ensure symmetry
    state_cov = (state_cov + state_cov.transpose()) / 2;

    // Update the previous orientation and velocity
    imu_state.orientation_null = imu_state.orientation;
    imu_state.velocity_null = imu_state.velocity;
}

// Propagate state
void MSCKF::propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    // Calculate time delta from last update
    double dt = imuMsg->header.stamp.toSec() - imu_state.timestamp.toSec();

    // Contruct global a_G and angular velocity in imu frame
    Eigen::Vector3d gyro_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_m = Eigen::Vector3d::Zero();
    tf::vectorMsgToEigen(msg->angular_velocity, gyro_m);
    tf::vectorMsgToEigen(msg->linear_acceleration, acc_m);

    propagateIMU(dt, gyro_m, acc_m);
    imu_state.timestamp = imuMsg->header.stamp;
}

// State augmentation
void MSCKF::stateAugmentation() {
    // Calculate rotation matrix from IMU to global frame
    Eigen::Matrix3d I_R_G = quaternionToRotation(imu_state.orientation);

    Eigen::Matrix3d C_R_G = imu_state.C_R_I *I_R_G
                                Eigen::Vector3d c_p_g = imu_state.position + I_R_G * imu_state.p_C;

    cam_state.q_G = imu_state.orientation * cam_state.q_I;
    cam_state.q_G.normalize();

    // Calculate rotation matrix from IMU to global frame
    Eigen::Matrix3d I_R_G = quaternionToRotation(imu_state.orientation);

    CAMERA::CAMState cam_state;
    cam_state.id = imu_state.id;
    cam_state.time = imu_state.timestamp.toSec();
    cam_state.orientation = rotationToQuaternion(C_R_G);
    cam_state.position = c_p_g;
    cam_state.orientation_null = cam_state.orientation;
    cam_state.position_null = cam_state.position;

    cam_states[imu_state.id] = cam_state;

    // Create the Jacobian
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 21);

    // Fill in Jacobian blocks
    J.block<3, 3>(0, 0) = imu_state.C_R_I;
    J.block<3, 3>(0, 15) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(3, 0) = skewMatrix(I_R_G.transpose() * imu_state.p_C);
    J.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(3, 18) = I_R_G.transpose();

    size_t old_rows = state_server.state_cov.rows();
    size_t old_cols = state_server.state_cov.cols();
    state_cov.conservativeResize(old_rows + 6, old_cols + 6);

    const Matrix<double, 21, 21> &P11 = state_cov.block<21, 21>(0, 0);
    const MatrixXd &P12 = state_cov.block(0, 21, 21, old_cols - 21);

    state_cov.block(old_rows, 0, 6, old_cols) << J * P11, J * P12;
    state_cov.block(0, old_cols, old_rows, 6) = state_cov.block(old_rows, 0, 6, old_cols).transpose();
    state_server.state_cov.block<6, 6>(old_rows, old_cols) = J * P11 * J.transpose();

    // Fix the covariance to be symmetric
    MatrixXd state_cov_fixed = (state_cov + state_cov.transpose()) / 2.0;
    state_cov = state_cov_fixed;
}
}  // namespace MSCKF