# ----- Pass/fail targets (from the MotionNode spec) -----
TARGETS = {
    "rate_tol": 0.01,          # sample rate within ±1 %
    "jitter_us": 100.0,        # timestamp jitter, RMS
    "skew_us": 1000.0,         # inter-IMU read skew
    "knee_rmse_deg": 3.0,      # knee-angle RMS error vs reference
}

# ----- Device settings (must match firmware) -----
FS_HZ = 200.0                  # firmware sample rate
ACCEL_LSB_PER_G = 4096.0       # ±8 g
GYRO_LSB_PER_DPS = 32.8        # ±1000 °/s
ACCEL_RANGE_G = 8.0
GYRO_RANGE_DPS = 1000.0
G = 9.80665

# ----- Sensor error model: MPU-6050 (datasheet typicals + assumptions) -----
SENSOR = {
    "gyro_nd_dps": 0.005,      # gyro noise density, °/s/√Hz (datasheet)
    "accel_nd_g": 400e-6,      # accel noise density, g/√Hz (datasheet)
    "gyro_bias_dps": 1.0,      # turn-on gyro bias, 1-sigma (assumed; datasheet ZRO is ±20 °/s max)
    "gyro_bias_rw": 0.01,      # gyro bias random walk, °/s per √s (assumed)
    "accel_bias_g": 0.03,      # accel zero-g offset, 1-sigma (assumed; datasheet ±50 mg typ)
    "scale_err": 0.02,         # sensitivity error, 1-sigma (datasheet ±3 % tolerance)
    "lpf_hz": 44.0,            # on-chip low-pass filter (CONFIG = 0x03)
}

# ----- Device timing (from the Wokwi run + assumptions) -----
DEVICE = {
    "skew_us": 768.0,          # measured inter-IMU read skew (Wokwi STATS line)
    "clock_ppm": 40.0,         # ESP32 crystal error vs true time (assumed; typical ±10–40 ppm)
    "jitter_us": 20.0,         # timestamp jitter on real hardware (assumed; Wokwi shows 0)
    "drop_rate": 0.001,        # random lost frames (assumed)
}

# ----- Mounting: how far each IMU is rotated from its segment (truth, unknown to the pipeline) -----
MOUNT_DEG = {"thigh": 6.0, "shank": -4.0}

# ----- Test protocol (device time, seconds): what the wearer does -----
PROTOCOL = {
    "static": (0.5, 4.5),      # stand still: gyro bias + mounting offset calibration
    "rigid": (5.5, 10.5),      # swing the leg with the knee locked: inter-IMU skew calibration
    "walk": (13.0, 73.0),      # 60 s of walking: accuracy evaluation
    "end_s": 75.0,
}

# ----- Body model (sagittal plane) -----
BODY = {
    "thigh_len_m": 0.45,
    "shank_len_m": 0.43,
    "thigh_sensor_m": 0.20,    # hip to thigh IMU
    "shank_sensor_m": 0.20,    # knee to shank IMU
    "stride_s": 1.1,           # walking stride period
    "stride_var": 0.03,        # stride-to-stride variability
    "bounce_m": 0.025,         # vertical hip bounce while walking
    "impact_g": 3.0,           # heel-strike shock at the shank, walking (running: 8-12 g)
}

# ----- Lab run settings -----
MONTE_CARLO_RUNS = 30
FS_INTERNAL = 1000.0           # simulation rate before the on-chip filter and decimation
