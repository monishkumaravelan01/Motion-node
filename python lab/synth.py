import numpy as np
from scipy.signal import butter, lfilter

from . import config as C


def _ramp(t, start, end, edge=0.4):
    up = 0.5 * (1 + np.tanh((t - start) / (edge / 3)))
    down = 0.5 * (1 - np.tanh((t - end) / (edge / 3)))
    return up * down


def _bump(phase, centre, width):
    d = (phase - centre + 0.5) % 1.0 - 0.5
    return np.exp(-0.5 * (d / width) ** 2)


def segmentAngles(t, rng, body=None, protocol=None):
    body = body or C.BODY
    protocol = protocol or C.PROTOCOL
    # stride period varies a little from stride to stride
    nStrides = int(t[-1] / body["stride_s"]) + 5
    periods = body["stride_s"] * (1 + body["stride_var"] * rng.standard_normal(nStrides))
    edges = np.concatenate([[0.0], np.cumsum(periods)])
    walkT = np.clip(t - (protocol["walk"][0] - 2.0), 0, None)   # gait already running under the ramp-in
    idx = np.searchsorted(edges, walkT, side="right") - 1
    phase = (walkT - edges[idx]) / periods[np.clip(idx, 0, nStrides - 1)]

    knee = 5 + 15 * _bump(phase, 0.15, 0.08) + 55 * _bump(phase, 0.73, 0.09)   # deg flexion
    thigh = 10 + 20 * np.cos(2 * np.pi * (phase - 0.88))

    wWalk = _ramp(t, *protocol["walk"])
    wRigid = _ramp(t, *protocol["rigid"])
    swing = 25 * np.sin(2 * np.pi * 1.2 * (t - protocol["rigid"][0]))           # knee locked

    thighDeg = wWalk * thigh + wRigid * swing
    shankDeg = wWalk * (thigh - knee) + wRigid * swing
    return thighDeg, shankDeg, phase, wWalk


def _sensorPointSpecificForce(t, thighDeg, shankDeg, wWalk, phase, body, impactG):
    dt = t[1] - t[0]
    th, sh = np.radians(thighDeg), np.radians(shankDeg)
    dTh = np.stack([np.sin(th), -np.cos(th)])            # thigh direction, pointing down
    dSh = np.stack([np.sin(sh), -np.cos(sh)])
    L_t, L_s = body["thigh_len_m"], body["shank_len_m"]
    rT, rS = body["thigh_sensor_m"], body["shank_sensor_m"]

    def acc(p):
        return np.gradient(np.gradient(p, dt, axis=1), dt, axis=1)

    # hip-anchored (swing): hip bounces vertically, forward speed is constant (no acceleration)
    hip = np.stack([np.zeros_like(t), wWalk * body["bounce_m"] * np.sin(2 * np.pi * 2 * phase)])
    aThSwing = acc(hip + rT * dTh)
    aShSwing = acc(hip + L_t * dTh + rS * dSh)
    # ankle-anchored (stance): foot planted
    knee = -L_s * dSh
    aShStance = acc(knee + rS * dSh)
    aThStance = acc(knee - (L_t - rT) * dTh)

    stance = (wWalk > 0.5) & (phase < 0.6)
    aTh = np.where(stance, aThStance, aThSwing)
    aSh = np.where(stance, aShStance, aShSwing)
    g = np.array([[0.0], [C.G]])                           # accelerometers feel +1 g upward at rest
    fTh, fSh = aTh + g, aSh + g

    # heel-strike shock on the shank: damped 30 Hz ring along the segment at each stride start
    strike = np.flatnonzero((np.diff(phase) < -0.5) & (wWalk[1:] > 0.5)) + 1
    shock = np.zeros_like(t)
    tau, fRing = 0.015, 30.0
    tt = np.arange(0, 0.1, dt)
    unitPeak = np.max(np.exp(-tt / tau) * np.sin(2 * np.pi * fRing * tt))   # so impactG is the peak
    for k in strike:
        seg = t[k:k + int(0.1 / dt)] - t[k]
        shock[k:k + len(seg)] += impactG / unitPeak * C.G * np.exp(-seg / tau) * np.sin(2 * np.pi * fRing * seg)
    fSh = fSh - shock * dSh                             # along the segment's up-axis
    return fTh, fSh


def _imuErrors(rng, sensor):
    return {
        "gyro_bias": sensor["gyro_bias_dps"] * rng.standard_normal(),
        "accel_bias": sensor["accel_bias_g"] * rng.standard_normal(2),
        "scale": 1 + sensor["scale_err"] * rng.standard_normal(3),   # ax, az, gy
    }


def _measure(t, phiDeg, fWorld, err, rng, sensor, fsInt):
    """One IMU: rotate specific force into the sensor frame, add errors, on-chip LPF."""
    phi = np.radians(phiDeg)
    fx = (np.cos(phi) * fWorld[0] - np.sin(phi) * fWorld[1]) / C.G
    fz = (np.sin(phi) * fWorld[0] + np.cos(phi) * fWorld[1]) / C.G
    omega = np.gradient(phiDeg, t)

    n = len(t)
    nyqNoise = np.sqrt(fsInt / 2)
    biasRw = np.cumsum(rng.standard_normal(n)) * sensor["gyro_bias_rw"] / np.sqrt(fsInt)
    gy = err["scale"][2] * omega + err["gyro_bias"] + biasRw \
        + sensor["gyro_nd_dps"] * nyqNoise * rng.standard_normal(n)
    ax = err["scale"][0] * fx + err["accel_bias"][0] + sensor["accel_nd_g"] * nyqNoise * rng.standard_normal(n)
    az = err["scale"][1] * fz + err["accel_bias"][1] + sensor["accel_nd_g"] * nyqNoise * rng.standard_normal(n)
    ay = sensor["accel_nd_g"] * nyqNoise * rng.standard_normal(n)
    gx = sensor["gyro_nd_dps"] * nyqNoise * rng.standard_normal(n)
    gz = sensor["gyro_nd_dps"] * nyqNoise * rng.standard_normal(n)

    # the ADC saturates at full scale BEFORE the on-chip digital filter: clipped data is lost for good
    raw = {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}
    clipped = int(sum((np.abs(raw[c]) >= C.ACCEL_RANGE_G).sum() for c in ("ax", "ay", "az")))
    for c in ("ax", "ay", "az"):
        raw[c] = np.clip(raw[c], -C.ACCEL_RANGE_G, C.ACCEL_RANGE_G)
    for c in ("gx", "gy", "gz"):
        raw[c] = np.clip(raw[c], -C.GYRO_RANGE_DPS, C.GYRO_RANGE_DPS)
    b, a = butter(2, sensor["lpf_hz"] / (fsInt / 2))
    out = {k: lfilter(b, a, v) for k, v in raw.items()}
    out["clipped"] = clipped
    return out


def _toCounts(ch):
    acc = np.clip(np.round(np.stack([ch["ax"], ch["ay"], ch["az"]], 1) * C.ACCEL_LSB_PER_G), -32768, 32767)
    gyr = np.clip(np.round(np.stack([ch["gx"], ch["gy"], ch["gz"]], 1) * C.GYRO_LSB_PER_DPS), -32768, 32767)
    return np.hstack([acc, gyr]).astype(np.int16)


def generate(seed=0, sensor=None, device=None, mount=None, body=None, protocol=None):
    rng = np.random.default_rng(seed)
    sensor = sensor or C.SENSOR
    device = device or C.DEVICE
    mount = mount or C.MOUNT_DEG
    body = body or C.BODY
    protocol = protocol or C.PROTOCOL
    fsInt = C.FS_INTERNAL

    t = np.arange(0, protocol["end_s"] + 1.0, 1 / fsInt)
    thigh, shank, phase, wWalk = segmentAngles(t, rng, body, protocol)
    fTh, fSh = _sensorPointSpecificForce(t, thigh, shank, wWalk, phase, body, body["impact_g"])

    # sensor pitch: the sensor's up-axis tilts opposite to segment flexion, plus the mounting offset
    phiTh = -thigh + mount["thigh"]
    phiSh = -shank + mount["shank"]
    err1, err2 = _imuErrors(rng, sensor), _imuErrors(rng, sensor)
    ch1 = _measure(t, phiTh, fTh, err1, rng, sensor, fsInt)
    ch2 = _measure(t, phiSh, fSh, err2, rng, sensor, fsInt)

    # device sampling: the ESP32 clock runs (1 + ppm) slow/fast vs true time; IMU2 is read skew later
    n = int(protocol["end_s"] * C.FS_HZ)
    k = np.arange(n)
    tDev = k / C.FS_HZ
    tTrue = tDev * (1 + device["clock_ppm"] * 1e-6)
    skew = device["skew_us"] * 1e-6
    clipped = ch2.pop("clipped") + ch1.pop("clipped")
    peakG = float(np.max(np.abs(fSh)) / C.G)
    imu1 = _toCounts({c: np.interp(tTrue, t, v) for c, v in ch1.items()})
    imu2 = _toCounts({c: np.interp(tTrue + skew, t, v) for c, v in ch2.items()})
    tUs = np.round(tDev * 1e6 + device["jitter_us"] * rng.standard_normal(n)).astype(np.int64)

    keep = rng.random(n) >= device["drop_rate"]           # frames lost in transport
    log = {
        "seq": k[keep], "t_us": tUs[keep], "skew_us": np.full(keep.sum(), int(device["skew_us"])),
        "imu1": imu1[keep], "imu2": imu2[keep], "flags": np.zeros(keep.sum(), dtype=int),
    }
    truth = {
        "t": t, "thigh": thigh, "shank": shank, "knee": thigh - shank,
        "mount": dict(mount), "skew_us": device["skew_us"], "clock_ppm": device["clock_ppm"],
        "gyro_bias": (err1["gyro_bias"], err2["gyro_bias"]),
        "dropped": int((~keep).sum()),
        "clipped_ms": clipped / C.FS_INTERNAL * 1000,      # time spent saturated at the ADC
        "shank_peak_g": peakG,
    }
    return log, truth
