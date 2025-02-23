#include <gtest/gtest.h>
#include <ros/ros.h>
#include "msckf/msckf.h"

class TestMSCKF : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        nh = ros::NodeHandle();
        pnh = ros::NodeHandle("~");
        msckf.reset(new MSCKF::Msckf(nh, pnh));
    }

    ros::NodeHandle nh, pnh;
    std::unique_ptr<MSCKF::Msckf> msckf;
};

TEST_F(TestMSCKF, StationaryTest)
{
    sensor_msgs::Imu imu_msg;
    msckf->imu_state.timestamp.sec = 0;
    msckf->imu_state.timestamp.nsec = 0;

    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nsec = 10000000;

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.linear_acceleration.z = 9.80665;

    auto initial_pos = msckf->imu_state.state.p;
    auto initial_vel = msckf->imu_state.state.v;
    auto initial_quat = msckf->imu_state.state.q;

    msckf->propagateState(boost::make_shared<const sensor_msgs::Imu>(imu_msg));

    // Compute the error quaternion
    Eigen::Quaterniond q_error = initial_quat.conjugate() * msckf->imu_state.state.q;
    q_error.normalize(); // Ensure normalization

    // Compute the rotation error (angle difference)
    double cos_theta = std::max(-1.0, std::min(q_error.w(), 1.0)); // Clamp to [-1,1]
    double angle_diff = 2.0 * std::acos(cos_theta);

    // Optionally convert to degrees
    double angle_diff_deg = angle_diff * 180.0 / M_PI;

    printf("Quaternion difference angle: %.6f radians (%.6f degrees)\n", angle_diff, angle_diff_deg);

    EXPECT_NEAR(msckf->imu_state.state.p.norm(), initial_pos.norm(), 1e-4);
    EXPECT_NEAR(msckf->imu_state.state.v.norm(), initial_vel.norm(), 5e-3);
    EXPECT_TRUE(msckf->imu_state.state.q.isApprox(initial_quat, 1));
}

TEST_F(TestMSCKF, CovarianceGrowthTest)
{
    Eigen::Matrix<double, 15, 15> initial_cov = msckf->P_II;

    sensor_msgs::Imu imu_msg;

    msckf->imu_state.timestamp.sec = 0;
    msckf->imu_state.timestamp.nsec = 0;

    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nsec = 10000000;

    imu_msg.angular_velocity.z = 0.1;
    imu_msg.linear_acceleration.x = 0.1;

    double dt = 0.01;
    msckf->propagateState(boost::make_shared<const sensor_msgs::Imu>(imu_msg));
    msckf->propagateCovariance(dt);

    EXPECT_GT(msckf->P_II.trace(), initial_cov.trace());
}

TEST_F(TestMSCKF, DynamicMotionTest)
{
    const double dt = 0.01; // timestep in seconds
    const int steps = 0.05;  // simulate 2 seconds of motion
    const double total_time = steps * dt;

    // Reset initial time and assume initial state (p = 0, v = 0, q = identity)
    msckf->imu_state.timestamp.sec = 0;
    msckf->imu_state.timestamp.nsec = 0;

    // Loop to simulate dynamic motion:
    for (int i = 0; i < steps; i++) {
        sensor_msgs::Imu imu_msg;

        // Update the timestamp for this message (convert dt to nanoseconds)
        imu_msg.header.stamp.sec = 0;
        imu_msg.header.stamp.nsec = (i + 1) * static_cast<int>(dt * 1e9);

        // Set constant angular velocity: 0.1 rad/s about the z-axis.
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.1;

        // Set constant linear acceleration.
        // Here, 1.0 m/s² in x plus gravity in z (as in the stationary test) so that the net effect is acceleration in x.
        imu_msg.linear_acceleration.x = 1.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 9.80665;

        // Propagate the filter state and covariance.
        msckf->propagateState(boost::make_shared<const sensor_msgs::Imu>(imu_msg));
        msckf->propagateCovariance(dt);
    }

    // ----- Compute theoretical expected state -----
    // Under the assumption that gravity is properly compensated,
    // the effective inertial acceleration is: a_inertial = R(t) * [1, 0, 0]
    // where R(t) rotates the body frame by an angle 0.1*t around z.
    // Thus, the analytical expressions become:
    //
    // Final orientation: a rotation of theta = 0.1 * total_time about the z-axis.
    double theta = 0.1 * total_time;
    Eigen::Quaterniond q_expected(cos(theta / 2.0), 0, 0, sin(theta / 2.0));

    // Expected velocity is given by:
    // v_x = ∫0^T cos(0.1*t) dt = sin(0.1*T)/0.1
    // v_y = ∫0^T sin(0.1*t) dt = (1 - cos(0.1*T))/0.1
    double v_x = sin(0.1 * total_time) / 0.1;
    double v_y = (1 - cos(0.1 * total_time)) / 0.1;
    Eigen::Vector3d v_expected(v_x, v_y, 0.0);

    // Expected position is the double integral of acceleration:
    // p_x = ∫0^T [∫0^t cos(0.1*s) ds] dt = (1 - cos(0.1*T))/(0.1*0.1)
    // p_y = ∫0^T [∫0^t sin(0.1*s) ds] dt = (0.1*T - sin(0.1*T))/(0.1*0.1)
    double p_x = (1 - cos(0.1 * total_time)) / (0.1 * 0.1);
    double p_y = (0.1 * total_time - sin(0.1 * total_time)) / (0.1 * 0.1);
    Eigen::Vector3d p_expected(p_x, p_y, 0.0);

    // ----- Compare the filter's final state to the theoretical values -----
    // Tolerances can be adjusted based on the numerical integration accuracy.
    double tol_position = 0.05; // meters
    double tol_velocity = 0.05; // m/s
    double tol_angle = 0.05;    // radians

    EXPECT_NEAR((msckf->imu_state.state.p - p_expected).norm(), 0, tol_position);
    EXPECT_NEAR((msckf->imu_state.state.v - v_expected).norm(), 0, tol_velocity);

    // Compute orientation error as the angle difference between quaternions.
    Eigen::Quaterniond q_error = q_expected.conjugate() * msckf->imu_state.state.q;
    q_error.normalize();
    double cos_theta_err = std::max(-1.0, std::min(q_error.w(), 1.0));
    double angle_diff = 2.0 * std::acos(cos_theta_err);
    EXPECT_NEAR(angle_diff, 0, tol_angle);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_msckf");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}