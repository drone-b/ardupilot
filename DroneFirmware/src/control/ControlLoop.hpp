#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"
#include "control/RateController.hpp"
#include "sensing/Sensor.hpp"

namespace dfw::control {

struct StateVector {
    float roll_rad {0.0f};
    float pitch_rad {0.0f};
    float yaw_rad {0.0f};
    bool valid {false};
};

struct MotionTarget {
    float roll_rate_target_rad_s {0.0f};
    float pitch_rate_target_rad_s {0.0f};
    float yaw_rate_target_rad_s {0.0f};
    float thrust_target {0.0f};
};

struct ControlDemand {
    float roll {0.0f};
    float pitch {0.0f};
    float yaw {0.0f};
    float thrust {0.0f};
};

struct AxisControlDebug {
    float rate_error_rad_s {0.0f};
    float proportional {0.0f};
    float integrator {0.0f};
    float derivative {0.0f};
    float unsaturated_output {0.0f};
    float saturated_output {0.0f};
    bool saturated {false};
};

struct ControlLoopDebug {
    AxisControlDebug roll {};
    AxisControlDebug pitch {};
    AxisControlDebug yaw {};
};

struct ControlUpdateResult {
    bool output_valid {false};
    sensing::ImuSampleStatus sample_status {sensing::ImuSampleStatus::no_data};
};

class ControlLoop {
public:
    explicit ControlLoop(platform::IHal& hal);

    void initialize();
    void update(const sensing::ImuSample& imu_sample,
                const MotionTarget& target,
                common::TimestampUs now_us);
    const ControlDemand& latest_demand() const;
    const ControlLoopDebug& latest_debug() const;
    float latest_dt_s() const;
    const ControlUpdateResult& latest_result() const;

private:
    static sensing::ImuSampleStatus validate_imu_sample(const sensing::ImuSample& imu_sample,
                                                        common::TimestampUs now_us);

    platform::IHal& hal_;
    RateController roll_controller_;
    RateController pitch_controller_;
    RateController yaw_controller_;
    ControlDemand latest_demand_ {};
    ControlLoopDebug latest_debug_ {};
    ControlUpdateResult latest_result_ {};
    common::TimestampUs previous_sample_time_us_ {0};
    float latest_dt_s_ {0.0f};
};

} // namespace dfw::control
