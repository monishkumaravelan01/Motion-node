"""MotionNode Lab metrics: log loading, integrity, timing, accuracy."""
import struct

import numpy as np

from . import config as C

FRAME_FMT = "<HIqH6h6hhB"       # must match the firmware Frame struct (before the CRC)
SYNC = 0xA55A


def crc16Ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def loadCsv(path):
    rows, malformed, crcFail = [], 0, 0
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("seq"):
                continue
            p = line.split(",")
            if len(p) != 18:
                malformed += 1
                continue
            try:
                v = [int(x) for x in p[:17]]
                crc = int(p[17], 16)
            except ValueError:
                malformed += 1
                continue
            if crc16Ccitt(struct.pack(FRAME_FMT, SYNC, *v)) != crc:
                crcFail += 1
                continue
            rows.append(v)
    a = np.array(rows, dtype=np.int64)
    log = {"seq": a[:, 0], "t_us": a[:, 1], "skew_us": a[:, 2],
           "imu1": a[:, 3:9], "imu2": a[:, 9:15], "flags": a[:, 16]}
    return log, {"malformed": malformed, "crc_fail": crcFail}


def integrity(log, extra=None):
    seq, flags = log["seq"], log["flags"]
    gaps = np.diff(seq) - 1
    out = {
        "frames": int(len(seq)),
        "lost": int(gaps[gaps > 0].sum()),
        "gap_events": int((gaps > 0).sum()),
        "overrun": int(((flags & 0x01) > 0).sum()),
        "imu1_err": int(((flags & 0x02) > 0).sum()),
        "imu2_err": int(((flags & 0x04) > 0).sum()),
        "dropped_flag": int(((flags & 0x08) > 0).sum()),
    }
    if extra:
        out.update(extra)
    return out


def timing(log):
    seq, t = log["seq"], log["t_us"].astype(float)
    consec = np.diff(seq) == 1
    dt = np.diff(t)[consec]
    slope = np.polyfit(seq, t, 1)[0]                       # µs per frame
    nominal = 1e6 / C.FS_HZ
    return {
        "rate_hz": 1e6 / dt.mean(),
        "jitter_us": float(dt.std()),
        "dt_min_us": float(dt.min()), "dt_max_us": float(dt.max()),
        "skew_mean_us": float(np.mean(log["skew_us"])), "skew_max_us": float(np.max(log["skew_us"])),
        "timer_vs_nominal_ppm": float((slope - nominal) / nominal * 1e6),
    }


def accuracy(est, ref):
    e = est - ref
    sd = float(e.std(ddof=1))
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))), "bias": float(e.mean()),
        "max_abs": float(np.abs(e).max()), "sd": sd,
        "loa_low": float(e.mean() - 1.96 * sd), "loa_high": float(e.mean() + 1.96 * sd),
    }
