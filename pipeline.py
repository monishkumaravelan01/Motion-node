import numpy as np
from scipy.signal import lfilter
from . import config as C

def toUnits(log):
    t = log["t_us"] * 1e-6
    seq = log["seq"]
    period = 1 / C.FS_HZ
    gridSeq = np.arange(seq[0], seq[-1] + 1)
    # timestamps of lost frames are reconstructed from the sequence number
    tGrid = np.interp(gridSeq, seq, t)
    out = {"t": tGrid, "gap": ~np.isin(gridSeq, seq), "period": period}
    for name in ("imu1", "imu2"):
        raw = log[name].astype(float)
        acc = raw[:, :3] / C.ACCEL_LSB_PER_G
        gyr = raw[:, 3:] / C.GYRO_LSB_PER_DPS
        out[name] = {
            "ax": np.interp(tGrid, t, acc[:, 0]), "az": np.interp(tGrid, t, acc[:, 2]),
            "gy": np.interp(tGrid, t, gyr[:, 1]),
        }
    return out


def window(d, span):
    return (d["t"] >= span[0]) & (d["t"] < span[1])


def accelPitch(imu):
    return np.degrees(np.arctan2(-imu["ax"], imu["az"]))


def calibrateStatic(d, protocol=None):
    protocol = protocol or C.PROTOCOL
    m = window(d, protocol["static"])
    cal = {}
    for name in ("imu1", "imu2"):
        cal[name] = {
            "gyro_bias": float(d[name]["gy"][m].mean()),
            "mount_deg": float(accelPitch({k: v[m] for k, v in d[name].items()}).mean()),
        }
    return cal


def estimateSkew(d, cal, protocol=None):
    protocol = protocol or C.PROTOCOL
    m = window(d, protocol["rigid"])
    t = d["t"][m]
    g1 = d["imu1"]["gy"][m] - cal["imu1"]["gyro_bias"]
    g2 = d["imu2"]["gy"][m] - cal["imu2"]["gyro_bias"]
    spec = np.abs(np.fft.rfft(g1 * np.hanning(len(g1))))
    freqs = np.fft.rfftfreq(len(g1), d["period"])
    f0 = freqs[1 + np.argmax(spec[1:])]
    # refine the frequency with a fine local search
    best = None
    for f in np.linspace(f0 - 0.05, f0 + 0.05, 101):
        X = np.column_stack([np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t), np.ones_like(t)])
        r = np.linalg.lstsq(X, g1, rcond=None)[1]
        if best is None or (len(r) and r[0] < best[0]):
            best = (r[0] if len(r) else 0, f)
    f0 = best[1]
    X = np.column_stack([np.sin(2 * np.pi * f0 * t), np.cos(2 * np.pi * f0 * t), np.ones_like(t)])
    c1 = np.linalg.lstsq(X, g1, rcond=None)[0]
    c2 = np.linalg.lstsq(X, g2, rcond=None)[0]
    dphi = np.angle(c2[0] + 1j * c2[1]) - np.angle(c1[0] + 1j * c1[1])
    dphi = (dphi + np.pi) % (2 * np.pi) - np.pi
    # IMU2 sampled later -> its signal leads in device time -> positive phase difference
    return float(dphi / (2 * np.pi * f0) * 1e6)


def shift(imu, t, delayS):
    return {k: np.interp(t - delayS, t, v) for k, v in imu.items()}


def complementary(phiAcc, omega, dt, tau=0.5):
    a = tau / (tau + dt)
    u = a * omega * dt + (1 - a) * phiAcc
    zi = np.array([a * phiAcc[0]])
    return lfilter([1.0], [1.0, -a], u, zi=zi)[0]


def kalman(phiAcc, omega, fNorm, dt, qAngle=1e-4, qBias=5e-7, rStatic=1.0,
           alpha0=100.0, w0=50.0, gateG=0.1):
    alpha = np.gradient(omega, dt)
    quiet = (np.abs(alpha) < alpha0) & (np.abs(omega) < w0) & (np.abs(fNorm - 1.0) < gateG)
    a, b = float(phiAcc[0]), 0.0                     # state: angle, residual bias
    p00, p01, p11 = 1.0, 0.0, 0.05 ** 2               # covariance
    out = np.empty_like(phiAcc)
    for k in range(len(phiAcc)):
        # predict: angle += (omega - bias) * dt
        a += (omega[k] - b) * dt
        p00 += dt * (dt * p11 - 2 * p01) + qAngle
        p01 -= dt * p11
        p11 += qBias
        # update with the gravity tilt only when the segment is quiet
        if quiet[k]:
            s = p00 + rStatic
            k0, k1 = p00 / s, p01 / s
            innov = phiAcc[k] - a
            a += k0 * innov
            b += k1 * innov
            p11 -= k1 * p01
            p01 -= k0 * p01
            p00 -= k0 * p00
        out[k] = a
    return out


def gyroOnly(phiAcc, omega, dt):
    return phiAcc[0] + np.cumsum(omega) * dt


def run(log, filt="kalman", compensateSkew=True, calibrate=True, protocol=None):
    protocol = protocol or C.PROTOCOL
    d = toUnits(log)
    cal = calibrateStatic(d, protocol)
    skewUs = estimateSkew(d, cal, protocol)
    if not calibrate:
        cal = {n: {"gyro_bias": 0.0, "mount_deg": 0.0} for n in ("imu1", "imu2")}
    imu2 = shift(d["imu2"], d["t"], skewUs * 1e-6) if compensateSkew else d["imu2"]

    seg = {}
    for name, imu in (("thigh", d["imu1"]), ("shank", imu2)):
        c = cal["imu1" if name == "thigh" else "imu2"]
        phiAcc = accelPitch(imu)
        omega = imu["gy"] - c["gyro_bias"]
        fNorm = np.hypot(imu["ax"], imu["az"])
        if filt == "kalman":
            phi = kalman(phiAcc, omega, fNorm, d["period"])
        elif filt == "complementary":
            phi = complementary(phiAcc, omega, d["period"])
        elif filt == "gyro":
            phi = gyroOnly(phiAcc, omega, d["period"])
        else:
            phi = phiAcc
        seg[name] = -(phi - c["mount_deg"])        # sensor pitch -> segment angle
    return {"t": d["t"], "gap": d["gap"], "thigh": seg["thigh"], "shank": seg["shank"],
            "knee": seg["thigh"] - seg["shank"], "cal": cal, "skew_us": skewUs, "units": d}


def alignToReference(estT, est, refT, ref, spans, searchMs=30.0):
    lags = []
    for span in spans:
        m = (estT >= span[0]) & (estT < span[1])
        def cost(L):
            return np.mean((est[m] - np.interp(estT[m] + L, refT, ref)) ** 2)
        coarse = np.arange(-searchMs, searchMs, 0.5) * 1e-3
        best = coarse[int(np.argmin([cost(L) for L in coarse]))]
        fine = best + np.arange(-0.5, 0.5, 0.02) * 1e-3
        lags.append(fine[int(np.argmin([cost(L) for L in fine]))])
    mid = [np.mean(s) for s in spans]
    ppm = (lags[1] - lags[0]) / (mid[1] - mid[0]) * 1e6
    offset = lags[0] - mid[0] * ppm * 1e-6
    return offset, ppm


def referenceOnDeviceGrid(estT, refT, ref, offset, ppm):
    return np.interp(estT * (1 + ppm * 1e-6) + offset, refT, ref)
