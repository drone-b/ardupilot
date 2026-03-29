# Sensors and Estimation Glossary

## Scope

This document summarizes sensor, state-estimation, and navigation terms frequently used in ArduPilot-style
autopilot systems and relevant to `DroneFirmware`.

## Inertial Sensing Terms

| Term | Meaning |
|---|---|
| IMU | Inertial Measurement Unit, usually combining gyroscopes and accelerometers. |
| INS | Inertial Navigation System. |
| Gyroscope | Sensor measuring angular rate. |
| Accelerometer | Sensor measuring specific force. |
| Delta angle | Integrated angular motion over a sample interval. |
| Delta velocity | Integrated acceleration over a sample interval. |
| Bias | Slowly varying offset in a sensor signal. |
| Gyro bias | Estimated angular-rate offset. |
| Scale factor | Multiplicative correction applied to a sensor. |
| Misalignment | Sensor-axis orientation error relative to the vehicle frame. |
| Temperature compensation | Correction for sensor drift caused by temperature. |
| Calibration | Process of estimating offsets, scales, or alignments. |
| Sample period | Time between sensor measurements. |
| Sample age | How old a sample is at use time. |
| Transport latency | Delay between measurement and software availability. |
| Vibration | Mechanical disturbance corrupting inertial measurement quality. |
| Clipping | Sensor output saturation because real motion exceeded measurable range. |

## Common Navigation Sensors

| Term | Meaning |
|---|---|
| GNSS | Global Navigation Satellite System. |
| GPS | Specific GNSS constellation, often used as a generic name. |
| Magnetometer | Sensor measuring magnetic field, often used for heading. |
| Compass | Practical use of magnetometer data for directional reference. |
| Barometer | Pressure sensor used for altitude estimation. |
| Airspeed sensor | Sensor estimating motion through air. |
| Optical flow | Measurement of apparent image motion used for velocity aiding. |
| Rangefinder | Sensor measuring distance to a nearby surface or object. |
| LiDAR | Laser-based ranging sensor. |
| Sonar | Acoustic ranging sensor. |
| Visual odometry | Motion estimation derived from camera imagery. |
| Motion capture / Vicon | External high-precision position and attitude measurement system. |
| Beacon ranging | Localization based on measured distance to fixed beacons. |

## Estimation Terms

| Term | Meaning |
|---|---|
| State estimator | Software that fuses measurements into vehicle state estimates. |
| AHRS | Attitude and Heading Reference System. |
| EKF | Extended Kalman Filter. |
| DCM | Direction Cosine Matrix representation and related estimation method. |
| Complementary filter | Lightweight fusion method blending fast inertial propagation with slower absolute correction. |
| Observer | Dynamic estimation algorithm reconstructing state from measurements. |
| Sensor fusion | Combining different sensors into a more robust or complete estimate. |
| Innovation | Difference between predicted measurement and actual measurement. |
| Residual | Error term used to assess estimator consistency. |
| Propagation | Estimator step that advances state using inertial motion. |
| Correction | Estimator step that pulls state back toward measured references. |
| Covariance | Filter quantity describing uncertainty and correlation. |
| Source selection | Choosing which measurements feed the active estimate. |
| Fusion source set | Group of sensors currently used by the estimator. |
| Redundancy | Use of multiple similar sensors or estimates for robustness. |
| Source switching | Moving from one aiding source to another while maintaining continuity. |

## Attitude and Motion-State Terms

| Term | Meaning |
|---|---|
| Attitude | Vehicle orientation in space. |
| Heading | Direction the vehicle faces relative to a horizontal reference. |
| Roll | Rotation around the vehicle longitudinal axis. |
| Pitch | Rotation around the lateral axis. |
| Yaw | Rotation around the vertical axis. |
| Angular rate | Rotational speed around an axis. |
| Linear acceleration | Change in linear velocity over time. |
| Velocity | Linear motion rate. |
| Position | Estimated location in a global or local frame. |
| Altitude | Vertical position relative to a chosen datum. |
| Climb rate | Vertical speed. |
| Body frame | Coordinate system attached to the vehicle. |
| Earth frame | Reference frame fixed to the environment or navigation basis. |
| Local frame | Position/velocity frame centered near the operating area. |

## Estimator Health and Validity Terms

| Term | Meaning |
|---|---|
| Health flag | Indicator that a sensor or estimator is operating normally. |
| Consistency check | Comparison between redundant estimates or sensors. |
| Estimator validity | Boolean or state indicating whether estimator output is trusted. |
| Lane switching | Selecting between multiple estimator instances or configurations. |
| Glitch | Short abnormal measurement disturbance. |
| Outlier rejection | Logic rejecting implausible measurements before fusion. |
| Loss of aiding | Condition where an estimator loses an external measurement source. |
| Dead reckoning | Continuing motion estimation after aiding loss using internal propagation only. |
| Drift | Accumulating error due to imperfect inertial integration or bias estimation. |
| Reset | Estimator state correction or re-alignment after fault or reference change. |

## Navigation Terms

| Term | Meaning |
|---|---|
| Origin | Reference geographic point used for local navigation coordinates. |
| Home position | Safety/navigation reference location, often used for return logic. |
| Waypoint navigation | Travel through predefined geographic targets. |
| Loiter point | Fixed location used for hold or orbit behavior. |
| Terrain estimate | Ground-elevation information used for clearance-aware control. |
| Surface tracking | Maintaining a fixed distance to terrain or another surface. |
| Relative altitude | Altitude measured relative to a selected reference such as home. |
| Absolute altitude | Altitude measured relative to a global datum. |
| Horizontal dilution / GPS quality | Quality metrics describing satellite geometry and positioning confidence. |

## Filtering Terms

| Term | Meaning |
|---|---|
| Low-pass filter | Filter that attenuates high-frequency noise. |
| High-pass filter | Filter that attenuates slow drift and low-frequency components. |
| Notch filter | Filter that suppresses a narrow frequency band. |
| Harmonic notch filter | Notch family tracking motor/rotor harmonics. |
| Bandwidth | Frequency range allowed by a filter or estimator component. |
| Cutoff frequency | Frequency where filtering begins to strongly attenuate signals. |
| Noise floor | Background measurement noise level. |

## Mapping Guidance for DroneFirmware

Useful mappings into `DroneFirmware`:

- sensor ownership and sampling -> `SensorManager`
- IMU sample timing contract -> `ImuSample`
- lightweight attitude estimation -> `AttitudeEstimator`
- future higher-grade fusion -> `estimation/` expansion

## Notes

- ArduPilot uses much richer estimation than the current `DroneFirmware` baseline.
- A glossary like this is helpful when deciding which concepts to adopt next without inheriting implementation complexity directly.
