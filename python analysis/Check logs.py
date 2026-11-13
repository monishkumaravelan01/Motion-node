import struct
import sys
import numpy as np

FRAME_FMT = "<HIqH6h6hhB"
SYNC = 0xA55A
FLAG_NAMES = {0x01: "overrun", 0x02: "imu1_err", 0x04: "imu2_err", 0x08: "dropped"}
TARGET_RATE_HZ = 200.0
RATE_TOL = 0.01          
JITTER_MAX_US = 100.0
SKEW_MAX_US = 1000.0

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, identical to the firmware implementation."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def parse(path):
    rows, malformed, crc_fail = [], 0, 0
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("seq"):
                continue
            parts = line.split(",")
            if len(parts) != 18:
                malformed += 1
                continue
            try:
                seq, t_us, skew = int(parts[0]), int(parts[1]), int(parts[2])
                imu = [int(x) for x in parts[3:15]]
                temp, flags, crc = int(parts[15]), int(parts[16]), int(parts[17], 16)
            except ValueError:
                malformed += 1
                continue
            payload = struct.pack(FRAME_FMT, SYNC, seq, t_us, skew, *imu, temp, flags)
            if crc16_ccitt(payload) != crc:
                crc_fail += 1
                continue
            rows.append((seq, t_us, skew, flags))
    return np.array(rows, dtype=np.int64), malformed, crc_fail

def main(path):
    data, malformed, crc_fail = parse(path)
    if len(data) < 3:
        sys.exit("Not enough valid frames to analyse.")

    seq, t_us, skew, flags = data.T
    gaps = np.diff(seq) - 1
    lost = int(gaps[gaps > 0].sum())
    dt = np.diff(t_us).astype(float)[np.diff(seq) == 1]
    rate = 1e6 / dt.mean()
    jitter = dt.std()

    print(f"Valid frames      : {len(data)}")
    print(f"Malformed lines   : {malformed}")
    print(f"CRC failures      : {crc_fail}")
    print(f"Sequence gaps     : {int((gaps > 0).sum())} ({lost} frames missing: lost in transport or rejected by CRC)")
    for bit, name in FLAG_NAMES.items():
        print(f"Flag {name:<12} : {int(((flags & bit) > 0).sum())} frames")
    print(f"Sample rate       : {rate:.2f} Hz (dt mean {dt.mean():.1f} us, min {dt.min():.0f}, max {dt.max():.0f})")
    print(f"Timing jitter     : {jitter:.1f} us RMS")
    print(f"Inter-IMU skew    : mean {skew.mean():.0f} us, max {skew.max()} us")

    checks = {
        "rate within ±1 %": abs(rate - TARGET_RATE_HZ) / TARGET_RATE_HZ <= RATE_TOL,
        f"jitter < {JITTER_MAX_US:.0f} us": jitter < JITTER_MAX_US,
        f"skew < {SKEW_MAX_US:.0f} us": skew.max() < SKEW_MAX_US,
        "no CRC failures": crc_fail == 0,
        "no lost frames": lost == 0,
        "no overruns": int(((flags & 0x01) > 0).sum()) == 0,
    }
    print("\nPass/fail:")
    for name, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    sys.exit(0 if all(checks.values()) else 1)

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "D:\\Motion node\\analysis\\logs.csv")
