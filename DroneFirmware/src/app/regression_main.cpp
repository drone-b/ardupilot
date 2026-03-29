#include "comms/AiControlLink.hpp"
#include "control/ControlAllocator.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "future/HardwareGraph.hpp"
#include "sensing/Sensor.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

bool almost_equal(float a, float b, float eps)
{
    const float diff = a - b;
    return diff < eps && diff > -eps;
}

int run_ai_link_tests()
{
    dfw::comms::AiControlLink link {};
    dfw::comms::AiWrenchCommandV1 cmd {};
    cmd.version = 1;
    cmd.seq = 10;
    cmd.t_cmd_us = 1000;
    cmd.tau[0] = 0.1f;
    cmd.tau[1] = 0.2f;
    cmd.tau[2] = 0.3f;
    cmd.thrust = 0.4f;

    if (!link.ingest_wrench(cmd, 2000, 3000)) {
        std::printf("FAIL ai_link: ingest rejected valid command\n");
        return 1;
    }

    if (!link.has_fresh_command(2500, 3000)) {
        std::printf("FAIL ai_link: fresh command not reported as fresh\n");
        return 1;
    }

    if (link.link_state(9000, 3000) != dfw::comms::AiLinkState::stale) {
        std::printf("FAIL ai_link: stale command not detected\n");
        return 1;
    }

    return 0;
}

int run_estimator_tests()
{
    dfw::estimation::AttitudeEstimator estimator {};
    dfw::sensing::ImuSample sample {};
    sample.sample_time_us = 2000;
    sample.sample_period_us = 2000;
    sample.accel_mps2[0] = 0.0f;
    sample.accel_mps2[1] = 0.0f;
    sample.accel_mps2[2] = -9.81f;
    sample.gyro_rad_s[0] = 0.0f;
    sample.gyro_rad_s[1] = 0.0f;
    sample.gyro_rad_s[2] = 0.0f;
    sample.valid = true;
    sample.status = dfw::sensing::ImuSampleStatus::ok;

    if (!estimator.update(sample)) {
        std::printf("FAIL estimator: valid sample rejected\n");
        return 1;
    }

    const dfw::estimation::AttitudeState state = estimator.state();
    const float q_norm =
        std::sqrt(state.q[0] * state.q[0] + state.q[1] * state.q[1] + state.q[2] * state.q[2] +
                  state.q[3] * state.q[3]);

    if (!almost_equal(q_norm, 1.0f, 0.01f)) {
        std::printf("FAIL estimator: quaternion norm out of bounds\n");
        return 1;
    }

    for (float covariance : state.covariance_diag) {
        if (covariance <= 0.0f || covariance > 4.0f) {
            std::printf("FAIL estimator: covariance diagonal out of bounds\n");
            return 1;
        }
    }

    return 0;
}

int run_allocator_tests()
{
    dfw::control::ControlAllocator allocator {};
    dfw::control::ControlDemand demand {};
    demand.roll = 0.8f;
    demand.pitch = 0.6f;
    demand.yaw = 0.2f;
    demand.thrust = 1.5f;

    const dfw::control::MotorOutputs outputs = allocator.allocate(demand);
    for (float value : outputs.values) {
        if (value < 0.0f || value > 1.0f) {
            std::printf("FAIL allocator: output out of [0,1]\n");
            return 1;
        }
    }

    return 0;
}

int run_hardware_graph_tests()
{
    dfw::future::HardwareGraph graph_a {};
    dfw::future::HardwareGraph graph_b {};

    const dfw::future::NodeDescriptor imu_node {
        1001,
        0xA1B2C3D4U,
        1,
        10,
        0,
        dfw::future::BusKind::spi,
        true,
    };
    const dfw::future::NodeDescriptor esc_node {
        1002,
        0xAA55AA55U,
        2,
        20,
        0,
        dfw::future::BusKind::dronecan,
        true,
    };

    (void) graph_a.add_node(imu_node);
    (void) graph_a.add_node(esc_node);
    (void) graph_b.add_node(esc_node);
    (void) graph_b.add_node(imu_node);

    const auto snap_a = graph_a.snapshot();
    const auto snap_b = graph_b.snapshot();

    if (snap_a.fingerprint_crc32 != snap_b.fingerprint_crc32) {
        std::printf("FAIL hw_graph: fingerprint not deterministic\n");
        return 1;
    }

    return 0;
}

} // namespace

int main()
{
    int failures = 0;
    failures += run_ai_link_tests();
    failures += run_estimator_tests();
    failures += run_allocator_tests();
    failures += run_hardware_graph_tests();

    if (failures != 0) {
        std::printf("Regression suite failed: %d test group(s)\n", failures);
        return 1;
    }

    std::printf("Regression suite passed\n");
    return 0;
}
