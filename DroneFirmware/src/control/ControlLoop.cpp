#include "control/ControlLoop.hpp"

namespace dfw::control {

namespace {

constexpr dfw::common::DurationUs k_max_sample_age_us = 5000;
constexpr dfw::common::DurationUs k_min_sample_period_us = 250;
constexpr dfw::common::DurationUs k_max_sample_period_us = 4000;
constexpr dfw::common::DurationUs k_max_transport_latency_us = 1000;
constexpr float k_min_dt_s = 0.00025f;
constexpr float k_max_dt_s = 0.004f;

RateGains make_roll_pitch_gains()
{
    return RateGains {
        0.18f,
        0.08f,
        0.002f,
        0.30f,
        1.0f,
    };
}

RateGains make_yaw_gains()
{
    return RateGains {
        0.12f,
        0.05f,
        0.0f,
        0.20f,
        1.0f,
    };
}

float clamp_dt(float dt_s)
{
    if (dt_s < k_min_dt_s) {
        return k_min_dt_s;
    }

    if (dt_s > k_max_dt_s) {
        return k_max_dt_s;
    }

    return dt_s;
}

AxisControlDebug make_axis_debug(const RateControllerDebug& controller_debug)
{
    AxisControlDebug debug {};
    debug.rate_error_rad_s = controller_debug.error;
    debug.proportional = controller_debug.proportional;
    debug.integrator = controller_debug.integrator;
    debug.derivative = controller_debug.derivative;
    debug.unsaturated_output = controller_debug.unsaturated_output;
    debug.saturated_output = controller_debug.saturated_output;
    debug.saturated = controller_debug.saturated;
    return debug;
}

} // namespace

ControlLoop::ControlLoop(platform::IHal& hal) :
    hal_(hal),
    roll_controller_(make_roll_pitch_gains()),
    pitch_controller_(make_roll_pitch_gains()),
    yaw_controller_(make_yaw_gains())
{
}

void ControlLoop::initialize()
{
    hal_.board_control().service_watchdog();
    roll_controller_.reset();
    pitch_controller_.reset();
    yaw_controller_.reset();
}

void ControlLoop::update(const sensing::ImuSample& imu_sample,
                         const MotionTarget& target,
                         common::TimestampUs now_us)
{
    latest_result_.sample_status = validate_imu_sample(imu_sample, now_us);
    if (latest_result_.sample_status != sensing::ImuSampleStatus::ok) {
        latest_demand_ = {};
        latest_debug_ = {};
        latest_result_.output_valid = false;
        latest_dt_s_ = 0.0f;
        roll_controller_.reset();
        pitch_controller_.reset();
        yaw_controller_.reset();
        return;
    }

    if (previous_sample_time_us_ != 0 && imu_sample.sample_time_us > previous_sample_time_us_) {
        const common::DurationUs delta_us =
            static_cast<common::DurationUs>(imu_sample.sample_time_us - previous_sample_time_us_);
        latest_dt_s_ = static_cast<float>(delta_us) * 1.0e-6f;
    } else if (imu_sample.sample_period_us > 0) {
        latest_dt_s_ = static_cast<float>(imu_sample.sample_period_us) * 1.0e-6f;
    } else {
        latest_dt_s_ = 0.0f;
    }

    latest_dt_s_ = clamp_dt(latest_dt_s_);
    previous_sample_time_us_ = imu_sample.sample_time_us;

    latest_demand_.roll = roll_controller_.update(
        target.roll_rate_target_rad_s,
        imu_sample.gyro_rad_s[0],
        latest_dt_s_);
    latest_demand_.pitch = pitch_controller_.update(
        target.pitch_rate_target_rad_s,
        imu_sample.gyro_rad_s[1],
        latest_dt_s_);
    latest_demand_.yaw = yaw_controller_.update(
        target.yaw_rate_target_rad_s,
        imu_sample.gyro_rad_s[2],
        latest_dt_s_);
    latest_demand_.thrust = target.thrust_target;
    latest_debug_.roll = make_axis_debug(roll_controller_.last_debug());
    latest_debug_.pitch = make_axis_debug(pitch_controller_.last_debug());
    latest_debug_.yaw = make_axis_debug(yaw_controller_.last_debug());
    latest_result_.output_valid = true;
}

const ControlDemand& ControlLoop::latest_demand() const
{
    return latest_demand_;
}

const ControlLoopDebug& ControlLoop::latest_debug() const
{
    return latest_debug_;
}

float ControlLoop::latest_dt_s() const
{
    return latest_dt_s_;
}

const ControlUpdateResult& ControlLoop::latest_result() const
{
    return latest_result_;
}

sensing::ImuSampleStatus ControlLoop::validate_imu_sample(const sensing::ImuSample& imu_sample,
                                                          common::TimestampUs now_us)
{
    if (!imu_sample.valid) {
        return sensing::ImuSampleStatus::no_data;
    }

    if (imu_sample.sample_time_us == 0 ||
        imu_sample.acquisition_time_us == 0 ||
        imu_sample.acquisition_time_us < imu_sample.sample_time_us ||
        now_us < imu_sample.sample_time_us) {
        return sensing::ImuSampleStatus::invalid_timestamp;
    }

    const common::DurationUs sample_age_us =
        static_cast<common::DurationUs>(now_us - imu_sample.sample_time_us);
    if (sample_age_us > k_max_sample_age_us) {
        return sensing::ImuSampleStatus::stale_sample;
    }

    if (imu_sample.sample_period_us != 0 &&
        (imu_sample.sample_period_us < k_min_sample_period_us ||
         imu_sample.sample_period_us > k_max_sample_period_us)) {
        return sensing::ImuSampleStatus::invalid_period;
    }

    if (imu_sample.transport_latency_us > k_max_transport_latency_us) {
        return sensing::ImuSampleStatus::excessive_latency;
    }

    return sensing::ImuSampleStatus::ok;
}

} // namespace dfw::control
