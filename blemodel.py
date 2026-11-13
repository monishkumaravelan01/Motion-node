import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ----- Data source (from firmware) -----
FRAME_BYTES = 45            # one frame = both IMUs + timestamp + seq + CRC
RATE_HZ = 200               # frames per second
BUFFER_FRAMES = 256         # firmware ring buffer (1.28 s)

# ----- BLE protocol constants (Bluetooth Core spec, 1M PHY) -----
ATT_HDR = 3                 # notification header (opcode + handle)
L2CAP_HDR = 4
LL_OVERHEAD = 10            # preamble 1 + access address 4 + header 2 + CRC 3 bytes
T_IFS_US = 150              # inter-frame space
EMPTY_PDU_US = 80           # empty acknowledgement packet from the central
US_PER_BYTE = 8             # 1 Mbit/s PHY

# ----- Assumptions (replace with measured values on hardware) -----
MAX_PKTS_PER_EVENT = 6      # many phone stacks allow ~4-7 link-layer packets per connection event
I_BASE_MA = 60.0            # ESP32 active + BLE controller idle, no sleep (assumed)
I_IMUS_MA = 8.0             # 2x MPU-6050 (~3.9 mA each, datasheet)
I_RADIO_MA = 120.0          # radio TX/RX current while on air (assumed)
EVENT_OVERHEAD_US = 400     # radio wake + sync + empty packets per connection event (assumed)
BATTERY_MAH = 500.0
USABLE_FRACTION = 0.8       # don't run a LiPo to empty

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def framesPerNotification(mtu):
    return (mtu - ATT_HDR) // FRAME_BYTES


def llPacketsPerNotification(mtu, dle):  # used by simulate()
    payload = framesPerNotification(mtu) * FRAME_BYTES + ATT_HDR + L2CAP_HDR
    maxLl = 251 if dle else 27
    return math.ceil(payload / maxLl)


def fragmentSizes(mtu, dle):
    payload = framesPerNotification(mtu) * FRAME_BYTES + ATT_HDR + L2CAP_HDR
    maxLl = 251 if dle else 27
    full, rest = divmod(payload, maxLl)
    return [maxLl] * full + ([rest] if rest else [])


def notificationAirtimeUs(mtu, dle):
    return sum((LL_OVERHEAD + size) * US_PER_BYTE + T_IFS_US + EMPTY_PDU_US + T_IFS_US
               for size in fragmentSizes(mtu, dle))


def linkCapacity(mtu, ciMs, dle):
    nFrames = framesPerNotification(mtu)
    if nFrames == 0:
        return 0.0
    frags = fragmentSizes(mtu, dle)
    avgPktUs = notificationAirtimeUs(mtu, dle) / len(frags)
    ciS = ciMs / 1000
    pktsByCount = MAX_PKTS_PER_EVENT
    pktsByAirtime = (ciMs * 1000 * 0.9) / avgPktUs        # leave 10 % of the interval idle
    pktRate = min(pktsByCount, pktsByAirtime) / ciS
    return pktRate / len(frags) * nFrames


def radioDuty(mtu, ciMs, dle):
    notifsPerS = RATE_HZ / framesPerNotification(mtu)
    dataUs = notifsPerS * notificationAirtimeUs(mtu, dle)
    eventUs = (1000 / ciMs) * EVENT_OVERHEAD_US
    return (dataUs + eventUs) / 1e6


def simulate(mtu, ciMs, pFail=0.0, outage=None, durationS=60.0, seed=1):
    rng = np.random.default_rng(seed)
    nPer = framesPerNotification(mtu)
    pkts = llPacketsPerNotification(mtu, dle=True)
    notifsPerEvent = MAX_PKTS_PER_EVENT // pkts
    period = 1.0 / RATE_HZ
    ci = ciMs / 1000.0

    queue = []                      # sample times of frames waiting in the firmware buffer
    latencies, dropped = [], 0
    nextFrameT, nextEventT = 0.0, rng.uniform(0, ci)

    while nextFrameT < durationS or nextEventT < durationS:
        if nextFrameT <= nextEventT and nextFrameT < durationS:
            if len(queue) >= BUFFER_FRAMES:
                dropped += 1        # same policy as firmware: buffer full -> drop new frame, flag it
            else:
                queue.append(nextFrameT)
            nextFrameT += period
        else:
            t = nextEventT
            inOutage = outage is not None and outage[0] <= t < outage[0] + outage[1]
            if not inOutage and rng.random() >= pFail:
                for _ in range(notifsPerEvent):
                    if len(queue) < nPer:
                        break       # only send full batches
                    batch, queue = queue[:nPer], queue[nPer:]
                    latencies.extend((t - s) * 1000 for s in batch)
            nextEventT += ci
            if nextEventT >= durationS and nextFrameT >= durationS:
                break
    return np.array(latencies), dropped


def batteryHours(airFrac):
    iAvg = I_BASE_MA + I_IMUS_MA + airFrac * I_RADIO_MA
    return BATTERY_MAH * USABLE_FRACTION / iAvg, iAvg


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    need = RATE_HZ
    print(f"Data rate: {RATE_HZ} Hz x {FRAME_BYTES} B = {RATE_HZ * FRAME_BYTES / 1000:.1f} kB/s "
          f"({RATE_HZ * FRAME_BYTES * 8 / 1000:.0f} kbit/s)\n")

    # 1. Throughput
    print("1. Throughput (frames/s the link can carry; need >= 200)")
    print(f"{'MTU':>4} {'DLE':>4} {'CI ms':>6} {'frames/notif':>12} {'capacity':>9} {'margin':>7}")
    configs = []
    for mtu in (23, 185, 247):
        for dle in (False, True):
            for ci in (7.5, 15, 30, 50):
                cap = linkCapacity(mtu, ci, dle)
                configs.append((mtu, dle, ci, cap))
                margin = cap / need if need else 0
                print(f"{mtu:>4} {str(dle):>4} {ci:>6} {framesPerNotification(mtu):>12} "
                      f"{cap:>9.0f} {margin:>6.1f}x {'OK' if cap >= need else 'TOO SLOW'}")
    print("  -> MTU 23 can't fit one 45-byte frame: MTU negotiation is mandatory.")
    print("  -> Without DLE, each notification is split into 27-byte fragments: it works, but with less margin.\n")

    # Chosen operating point
    mtu, ci = 247, 15
    cap = linkCapacity(mtu, ci, True)
    print(f"Chosen: MTU {mtu}, DLE on, CI {ci} ms -> {framesPerNotification(mtu)} frames per "
          f"notification, capacity {cap:.0f} frames/s ({cap / need:.1f}x margin), "
          f"radio on {radioDuty(mtu, ci, True) * 100:.1f} % of the time\n")

    # 2. Latency
    lat, _ = simulate(mtu, ci)
    print("2. Latency (sample -> delivered), clean link")
    print(f"  mean {lat.mean():.1f} ms, p95 {np.percentile(lat, 95):.1f} ms, max {lat.max():.1f} ms")
    print("  -> dominated by batching (5 frames = 25 ms) plus waiting for the next connection event.\n")

    # 3. Loss vs interference and outages
    print("3. Frames lost (buffer overflow) vs interference and dropouts, 60 s run")
    lossRows = []
    for p in (0.0, 0.1, 0.3, 0.5):
        latP, drop = simulate(mtu, ci, pFail=p)
        lossRows.append((f"{int(p * 100)} % events lost", drop, np.percentile(latP, 95)))
    for outS in (0.5, 1.0, 2.0, 3.0):
        latO, drop = simulate(mtu, ci, outage=(20.0, outS))
        lossRows.append((f"{outS:.1f} s dropout", drop, np.percentile(latO, 95)))
    for name, drop, p95 in lossRows:
        print(f"  {name:<18} dropped {drop:>5} frames   p95 latency {p95:7.1f} ms")
    print(f"  -> Random interference is absorbed by retries + spare capacity. A dropout longer than the\n"
          f"     buffer ({BUFFER_FRAMES / RATE_HZ:.2f} s) loses data: buffer size sets the longest tolerable dropout.\n")

    # 4. Battery
    print("4. Battery (assumed currents, 500 mAh, 80 % usable)")
    for ciB in (7.5, 15, 30, 50):
        duty = radioDuty(mtu, ciB, True)
        hours, iAvg = batteryHours(duty)
        print(f"  CI {ciB:>4} ms: radio on {duty * 100:4.1f} %  avg {iAvg:5.1f} mA -> {hours:4.1f} h")
    hoursOff, iOff = batteryHours(0.0)
    print(f"  radio off (log to SD): avg {iOff:5.1f} mA -> {hoursOff:4.1f} h")
    print("  -> Shorter connection intervals cost more (every event wakes the radio), but the ESP32's base\n"
          "     current dominates. Biggest win = CPU light-sleep between samples. Must be measured on hardware.\n")

    # ----- Plots -----
    fig, ax = plt.subplots(figsize=(7, 4))
    for mtuP in (185, 247):
        cis = [c for (m, d, c, _) in configs if m == mtuP and d]
        caps = [cp for (m, d, c, cp) in configs if m == mtuP and d]
        ax.plot(cis, caps, marker="o", label=f"MTU {mtuP}, DLE on")
    ax.axhline(need, color="red", linestyle="--", label="Required 200 frames/s")
    ax.set_xlabel("Connection interval (ms)")
    ax.set_ylabel("Link capacity (frames/s)")
    ax.set_title("BLE throughput vs connection interval")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "ble_throughput.png"), dpi=150)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(lat, bins=40)
    ax.set_xlabel("Latency, sample to delivery (ms)")
    ax.set_ylabel("Frames")
    ax.set_title(f"Latency distribution (MTU {mtu}, CI {ci} ms, clean link)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "ble_latency.png"), dpi=150)

    outs = np.linspace(0.25, 3.0, 12)
    drops = [simulate(mtu, ci, outage=(20.0, o))[1] for o in outs]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(outs, drops, marker="o")
    ax.axvline(BUFFER_FRAMES / RATE_HZ, color="red", linestyle="--", label="Buffer length (1.28 s)")
    ax.set_xlabel("Radio dropout length (s)")
    ax.set_ylabel("Frames lost")
    ax.set_title("Data loss vs dropout length (256-frame buffer)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "ble_dropout_loss.png"), dpi=150)
    print(f"Plots saved to {OUT_DIR}")


if __name__ == "__main__":
    main()