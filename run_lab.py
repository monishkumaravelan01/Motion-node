import argparse
import copy
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from . import config as C
from . import metrics, pipeline, synth

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
EVAL = (C.PROTOCOL["walk"][0] + 2.0, C.PROTOCOL["walk"][1] - 1.0)      # skip walk ramp-in/out
ALIGN_SPANS = [(EVAL[0], EVAL[0] + 15), (EVAL[1] - 15, EVAL[1])]


def evaluate(log, truth, filt="kalman", compensateSkew=True, calibrate=True):
    r = pipeline.run(log, filt=filt, compensateSkew=compensateSkew, calibrate=calibrate)
    off, ppm = pipeline.alignToReference(r["t"], r["knee"], truth["t"], truth["knee"], ALIGN_SPANS)
    ref = pipeline.referenceOnDeviceGrid(r["t"], truth["t"], truth["knee"], off, ppm)
    m = (r["t"] >= EVAL[0]) & (r["t"] < EVAL[1])
    acc = metrics.accuracy(r["knee"][m], ref[m])
    return acc, r, {"offset_ms": off * 1e3, "ppm": ppm}, (r["t"][m], r["knee"][m], ref[m])


def idealConfig():
    sensor = {k: 0.0 for k in C.SENSOR}
    sensor["lpf_hz"] = C.SENSOR["lpf_hz"]
    device = {"skew_us": 0.0, "clock_ppm": 0.0, "jitter_us": 0.0, "drop_rate": 0.0}
    mount = {"thigh": 0.0, "shank": 0.0}
    return sensor, device, mount


def meanRmse(seeds, **genKw):
    evalKw = genKw.pop("evalKw", {})
    vals = []
    for s in seeds:
        log, truth = synth.generate(seed=s, **genKw)
        vals.append(evaluate(log, truth, **evalKw)[0]["rmse"])
    return float(np.mean(vals))


def errorBudget(seeds=(11, 12, 13)):
    sensor0, device0, mount0 = idealConfig()
    bodyQuiet = dict(C.BODY, impact_g=0.0)
    rows = [("Method only (ideal sensor)", meanRmse(seeds, sensor=sensor0, device=device0,
                                                     mount=mount0, body=bodyQuiet))]

    def one(label, sensorKw=None, deviceKw=None, mount=None, body=None, evalKw=None):
        s = dict(sensor0, **(sensorKw or {}))
        d = dict(device0, **(deviceKw or {}))
        rows.append((label, meanRmse(seeds, sensor=s, device=d, mount=mount or mount0,
                                      body=body or bodyQuiet, evalKw=evalKw or {})))

    one("Sensor noise (datasheet)", {"gyro_nd_dps": C.SENSOR["gyro_nd_dps"], "accel_nd_g": C.SENSOR["accel_nd_g"]})
    one("Gyro bias, calibrated", {"gyro_bias_dps": C.SENSOR["gyro_bias_dps"]})
    one("Gyro bias, NOT calibrated", {"gyro_bias_dps": C.SENSOR["gyro_bias_dps"]}, evalKw={"calibrate": False})
    one("Gyro bias drift", {"gyro_bias_rw": C.SENSOR["gyro_bias_rw"]})
    one("Scale error (2 %)", {"scale_err": C.SENSOR["scale_err"]})
    one("Accel bias (30 mg)", {"accel_bias_g": C.SENSOR["accel_bias_g"]})
    one("Mounting offset, calibrated", mount=C.MOUNT_DEG)
    one("Mounting offset, NOT calibrated", mount=C.MOUNT_DEG, evalKw={"calibrate": False})
    one("Skew 768 us, compensated", deviceKw={"skew_us": C.DEVICE["skew_us"]})
    one("Skew 768 us, NOT compensated", deviceKw={"skew_us": C.DEVICE["skew_us"]},
        evalKw={"compensateSkew": False})
    one("Clock drift 40 ppm", deviceKw={"clock_ppm": C.DEVICE["clock_ppm"]})
    one("Lost frames (0.1 %)", deviceKw={"drop_rate": C.DEVICE["drop_rate"]})
    one("Heel-strike impacts (3 g)", body=C.BODY)
    return rows


def monteCarlo(n):
    rmses, errs = [], []
    for s in range(100, 100 + n):
        log, truth = synth.generate(seed=s)
        acc, _, _, (t, est, ref) = evaluate(log, truth)
        rmses.append(acc["rmse"])
        errs.append(est - ref)
    errs = np.concatenate(errs)
    return np.array(rmses), errs


def requirementSweeps(seeds=(21, 22, 23)):
    noiseX = [1, 5, 20, 50, 100, 200]
    noise = []
    for x in noiseX:
        s = dict(C.SENSOR, gyro_nd_dps=C.SENSOR["gyro_nd_dps"] * x, accel_nd_g=C.SENSOR["accel_nd_g"] * x)
        noise.append(meanRmse(seeds, sensor=s))
    skews = [0, 500, 1000, 2000, 3000, 5000]
    skew = [meanRmse(seeds, device=dict(C.DEVICE, skew_us=float(k)),
                      evalKw={"compensateSkew": False}) for k in skews]
    return noiseX, noise, skews, skew


def clippingCheck():
    out = {}
    for label, g in (("walking 3 g", 3.0), ("running 12 g", 12.0)):
        _, truth = synth.generate(seed=5, body=dict(C.BODY, impact_g=g))
        out[label] = {"peak_g": round(truth["shank_peak_g"], 1), "saturated_ms": round(truth["clipped_ms"], 1)}
    return out


def plots(trace, acc, rmses, errs, budget, sweeps):
    os.makedirs(OUT, exist_ok=True)
    t, est, ref = trace
    m = (t >= t[0] + 20) & (t < t[0] + 25)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(t[m], ref[m], label="Reference (truth)", lw=2)
    ax.plot(t[m], est[m], "--", label="MotionNode estimate")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Knee flexion (deg)")
    ax.set_title(f"Knee angle, 5 s of walking (RMSE {acc['rmse']:.2f}°)")
    ax.legend(); ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "knee_trace.png"), dpi=150)

    fig, ax = plt.subplots(figsize=(6, 4.5))
    mean = (est + ref) / 2
    diff = est - ref
    ax.scatter(mean[::5], diff[::5], s=4, alpha=0.4)
    for y, lbl in ((acc["bias"], "bias"), (acc["loa_low"], "-1.96 SD"), (acc["loa_high"], "+1.96 SD")):
        ax.axhline(y, color="red" if lbl == "bias" else "gray", linestyle="-" if lbl == "bias" else "--")
        ax.text(mean.max(), y, f" {lbl} {y:.2f}°", va="center", fontsize=8)
    ax.set_xlabel("Mean of estimate and reference (deg)"); ax.set_ylabel("Estimate - reference (deg)")
    ax.set_title("Bland-Altman agreement"); ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "bland_altman.png"), dpi=150)

    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.hist(rmses, bins=12)
    ax.axvline(C.TARGETS["knee_rmse_deg"], color="red", linestyle="--", label="Target 3°")
    ax.set_xlabel("Knee RMSE per run (deg)"); ax.set_ylabel("Runs")
    ax.set_title(f"Monte Carlo, {len(rmses)} runs"); ax.legend(); ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "monte_carlo.png"), dpi=150)

    labels, vals = zip(*budget)
    fig, ax = plt.subplots(figsize=(8, 5))
    colors = ["tab:red" if "NOT" in l else "tab:blue" for l in labels]
    ax.barh(labels, vals, color=colors)
    ax.axvline(C.TARGETS["knee_rmse_deg"], color="red", linestyle="--", label="Target 3°")
    ax.invert_yaxis(); ax.set_xlabel("Knee RMSE with only this error source (deg)")
    ax.set_title("Error budget (red = without the calibration step)"); ax.legend()
    ax.grid(alpha=0.3, axis="x"); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "error_budget.png"), dpi=150)

    noiseX, noise, skews, skew = sweeps
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
    axs[0].plot(noiseX, noise, marker="o")
    axs[0].set_xscale("log"); axs[0].set_xlabel("Sensor noise (x MPU-6050 datasheet)")
    axs[1].plot(np.array(skews) / 1000, skew, marker="o")
    axs[1].set_xlabel("Inter-IMU skew, uncompensated (ms)")
    for a in axs:
        a.axhline(C.TARGETS["knee_rmse_deg"], color="red", linestyle="--")
        a.set_ylabel("Knee RMSE (deg)"); a.grid(alpha=0.3)
    axs[0].set_title("Requirement: sensor noise"); axs[1].set_title("Requirement: timing skew")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "requirements.png"), dpi=150)


def runSynthetic():
    os.makedirs(OUT, exist_ok=True)
    tgt = C.TARGETS
    print("MotionNode Lab: synthetic validation (reference = known truth)\n")

    # 1. one session: integrity, timing, calibration, accuracy
    log, truth = synth.generate(seed=1)
    integ = metrics.integrity(log)
    tim = metrics.timing(log)
    acc, r, align, trace = evaluate(log, truth)
    cal = r["cal"]

    # 2. filter comparison over 4 sessions
    filters = {}
    for f in ("accel", "gyro", "complementary", "kalman"):
        filters[f] = float(np.mean([evaluate(*synth.generate(seed=s), filt=f)[0]["rmse"] for s in (1, 2, 3, 4)]))

    # 3-6. Monte Carlo, error budget, requirements, clipping
    rmses, errs = monteCarlo(C.MONTE_CARLO_RUNS)
    budget = errorBudget()
    sweeps = requirementSweeps()
    clip = clippingCheck()
    plots(trace, acc, rmses, errs, budget, sweeps)

    uK2 = 2 * float(errs.std(ddof=1))
    p95 = float(np.percentile(rmses, 95))
    passRate = float(np.mean(rmses < tgt["knee_rmse_deg"]))
    checks = {
        "sample rate within ±1 %": abs(tim["rate_hz"] - C.FS_HZ) / C.FS_HZ <= tgt["rate_tol"],
        f"jitter < {tgt['jitter_us']:.0f} µs": tim["jitter_us"] < tgt["jitter_us"],
        f"skew < {tgt['skew_us']:.0f} µs": tim["skew_max_us"] < tgt["skew_us"],
        "lost frames detected (seq gaps = injected)": integ["lost"] == truth["dropped"],
        f"knee RMSE < {tgt['knee_rmse_deg']}° (mean of {len(rmses)} runs)": rmses.mean() < tgt["knee_rmse_deg"],
        f"knee RMSE < {tgt['knee_rmse_deg']}° (95th percentile)": p95 < tgt["knee_rmse_deg"],
    }

    summary = {
        "integrity": integ, "timing": tim, "alignment": align,
        "calibration": {"estimated": cal, "skew_us": r["skew_us"],
                        "truth": {"gyro_bias": truth["gyro_bias"], "mount": truth["mount"],
                                  "skew_us": truth["skew_us"], "clock_ppm": truth["clock_ppm"]}},
        "accuracy_session1": acc, "filters_mean_rmse": filters,
        "monte_carlo": {"runs": len(rmses), "rmse_mean": float(rmses.mean()), "rmse_p95": p95,
                        "rmse_max": float(rmses.max()), "pass_rate": passRate,
                        "error_bias": float(errs.mean()), "U_k2": uK2},
        "error_budget": budget, "requirements": {"noise_x": sweeps[0], "noise_rmse": sweeps[1],
                                                 "skew_us": sweeps[2], "skew_rmse": sweeps[3]},
        "clipping_8g": clip, "checks": checks,
    }
    with open(os.path.join(OUT, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2, default=float)
    writeReport(summary)

    for name, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    print(f"\nKnee angle: {errs.mean():+.2f}° ± {uK2:.2f}° (k=2), Monte Carlo {len(rmses)} runs")
    print(f"Report: {os.path.join(OUT, 'lab_report.md')}")
    return 0 if all(checks.values()) else 1


def writeReport(s):
    cal, tr = s["calibration"]["estimated"], s["calibration"]["truth"]
    mc, tim, integ, acc = s["monte_carlo"], s["timing"], s["integrity"], s["accuracy_session1"]
    L = ["# MotionNode Lab report", "",
         "Synthetic validation: the reference is the known true knee angle of a simulated session "
         "(stand 4 s, rigid leg swing 5 s, walk 60 s). On hardware the reference would be optical "
         "motion capture or an encoder hinge rig. All error sources are listed in lab/config.py.", "",
         "## Result", "",
         f"**Knee flexion error: {mc['error_bias']:+.2f}° ± {mc['U_k2']:.2f}° (k=2, ~95 %)**, "
         f"RMSE mean {mc['rmse_mean']:.2f}°, 95th percentile {mc['rmse_p95']:.2f}°, worst {mc['rmse_max']:.2f}° "
         f"over {mc['runs']} Monte Carlo runs; {mc['pass_rate'] * 100:.0f} % of runs meet the 3° target.", "",
         "| Check | Result |", "| --- | --- |"]
    L += [f"| {k} | {'PASS' if v else 'FAIL'} |" for k, v in s["checks"].items()]
    L += ["", "## Data integrity and timing (session 1)", "",
          "| Metric | Value |", "| --- | --- |",
          f"| Frames | {integ['frames']} |", f"| Lost frames (sequence gaps) | {integ['lost']} |",
          f"| Sample rate | {tim['rate_hz']:.2f} Hz |", f"| Jitter (RMS) | {tim['jitter_us']:.1f} µs |",
          f"| Inter-IMU read skew | {tim['skew_mean_us']:.0f} µs |",
          f"| Clock drift vs reference (estimated) | {s['alignment']['ppm']:.1f} ppm (injected {tr['clock_ppm']:.0f}) |",
          f"| Time offset vs reference | {s['alignment']['offset_ms']:.2f} ms (includes on-chip filter delay) |",
          "", "## Calibration: estimated vs injected", "",
          "| Quantity | Estimated | Injected |", "| --- | --- | --- |",
          f"| Thigh gyro bias | {cal['imu1']['gyro_bias']:.3f} °/s | {tr['gyro_bias'][0]:.3f} °/s |",
          f"| Shank gyro bias | {cal['imu2']['gyro_bias']:.3f} °/s | {tr['gyro_bias'][1]:.3f} °/s |",
          f"| Thigh mounting offset | {cal['imu1']['mount_deg']:.2f}° | {tr['mount']['thigh']:.2f}° |",
          f"| Shank mounting offset | {cal['imu2']['mount_deg']:.2f}° | {tr['mount']['shank']:.2f}° |",
          f"| Inter-IMU sample skew | {s['calibration']['skew_us']:.0f} µs | {tr['skew_us']:.0f} µs |",
          "", "Mounting-offset error comes from accelerometer bias (30 mg ≈ 1.7°): static calibration "
          "is limited by the accelerometer it relies on.", "",
          "## Sensor fusion comparison (mean knee RMSE, 4 sessions)", "",
          "| Method | RMSE |", "| --- | --- |"]
    L += [f"| {k} | {v:.2f}° |" for k, v in s["filters_mean_rmse"].items()]
    L += ["", "Accelerometer tilt alone is corrupted by segment acceleration during gait; a fixed-gain "
          "complementary filter lets that corruption in; gyro integration drifts with bias; the gated "
          "Kalman filter trusts gravity only when the segment is quiet and tracks the residual bias.", "",
          "## Error budget (one source at a time)", "", "| Source | Knee RMSE |", "| --- | --- |"]
    L += [f"| {k} | {v:.2f}° |" for k, v in s["error_budget"]]
    req = s["requirements"]
    L += ["", "## Requirements derived", "",
          "| Sensor noise (× datasheet) | Knee RMSE |", "| --- | --- |"]
    L += [f"| {x} | {v:.2f}° |" for x, v in zip(req["noise_x"], req["noise_rmse"])]
    L += ["", "| Uncompensated skew | Knee RMSE |", "| --- | --- |"]
    L += [f"| {k / 1000:.1f} ms | {v:.2f}° |" for k, v in zip(req["skew_us"], req["skew_rmse"])]
    L += ["", "## Range check (±8 g)", "", "| Scenario | Peak shank specific force | Time saturated at the ADC (60 s) |",
          "| --- | --- | --- |"]
    L += [f"| {k} | {v['peak_g']} g | {v['saturated_ms']} ms |" for k, v in s["clipping_8g"].items()]
    L += ["", "## Plots", "", "![knee](knee_trace.png)", "![ba](bland_altman.png)",
          "![mc](monte_carlo.png)", "![budget](error_budget.png)", "![req](requirements.png)", "",
          "## Limitations", "",
          "- Sagittal-plane (2-D) leg model; real gait is 3-D with soft-tissue motion.",
          "- Sensor errors are datasheet typicals plus stated assumptions, not measured units.",
          "- Zero-jitter Wokwi timing is replaced here by an assumed 20 µs; real values need hardware.",
          "- The reference is simulated truth; hardware validation needs mocap or an encoder rig."]
    with open(os.path.join(OUT, "lab_report.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")


def runLog(path):
    log, extra = metrics.loadCsv(path)
    integ = metrics.integrity(log, extra)
    tim = metrics.timing(log)
    print("MotionNode Lab: log check\n")
    for k, v in {**integ, **tim}.items():
        print(f"  {k:<22} {v:.2f}" if isinstance(v, float) else f"  {k:<22} {v}")
    checks = {
        "rate within ±1 %": abs(tim["rate_hz"] - C.FS_HZ) / C.FS_HZ <= C.TARGETS["rate_tol"],
        "jitter": tim["jitter_us"] < C.TARGETS["jitter_us"],
        "skew": tim["skew_max_us"] < C.TARGETS["skew_us"],
        "no CRC failures": integ["crc_fail"] == 0,
        "no lost frames": integ["lost"] == 0,
        "no overruns": integ["overrun"] == 0,
    }
    print()
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="MotionNode Lab")
    ap.add_argument("--log", help="firmware CSV log to check (integrity + timing)")
    args = ap.parse_args()
    sys.exit(runLog(args.log) if args.log else runSynthetic())
