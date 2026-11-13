#pragma once
#include <Arduino.h>
#include <esp_timer.h>
#include <ImuConfig.h>

// --- Timestamping, buffering, synchronization ---
// A hardware timer ticks every 5 ms and wakes a high-priority sampler task.
// The sampler reads both IMUs, timestamps them, packs a frame with seq + CRC
// and pushes it into a ring buffer. A low-priority writer task prints frames,
// so slow output never shifts the sampling rhythm.

// ----- Sampling and output config -----
constexpr uint32_t SAMPLE_PERIOD_US = 5000;   // 200 Hz
constexpr uint32_t STATS_EVERY      = 200;    // timing report once per second
constexpr uint32_t QUEUE_DEPTH      = 256;    // ring buffer: 256 frames = 1.28 s at 200 Hz
constexpr uint16_t FRAME_SYNC       = 0xA55A; // lets a binary parser find frame starts

// ----- Frame flags -----
constexpr uint8_t FLAG_OVERRUN  = 0x01;  // sampler missed one or more timer ticks
constexpr uint8_t FLAG_IMU1_ERR = 0x02;  // IMU1 read failed (data zeroed)
constexpr uint8_t FLAG_IMU2_ERR = 0x04;  // IMU2 read failed (data zeroed)
constexpr uint8_t FLAG_DROPPED  = 0x08;  // ring buffer overflowed since the previous frame

// 45-byte frame. Layout must match analysis/check_log.py (FRAME_FMT "<HIqH6h6hhB" + CRC)
struct __attribute__((packed)) Frame {
  uint16_t sync;      // 0xA55A
  uint32_t seq;       // contiguous per frame produced; a gap = lost after sampling
  int64_t  t_us;      // timestamp at start of IMU1 read (us since boot)
  uint16_t skew_us;   // start of IMU2 read minus start of IMU1 read
  int16_t  imu1[6];   // ax ay az gx gy gz (raw counts, converted later in Python)
  int16_t  imu2[6];
  int16_t  temp1;     // IMU1 die temperature (raw), for temperature-drift work
  uint8_t  flags;
  uint16_t crc;       // CRC-16 over every byte before it
};
static_assert(sizeof(Frame) == 45, "Frame layout changed: update check_log.py");

// One window of timing statistics, handed from the sampler to the writer
struct Stats {
  uint32_t n, overruns, drops, readMaxUs;
  double dtMean, dtMin, dtMax, jitter, skewMean;
};

static QueueHandle_t frameQueue = nullptr;
static TaskHandle_t samplerTask = nullptr;
static Stats statsSnap;
static volatile bool statsReady = false;   // sampler sets it, writer clears it after printing

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF (same as check_log.py)
inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Runs every 5 ms. Only wakes the sampler: no I2C, no printing here.
static void onSampleTimer(void *arg) {
  xTaskNotifyGive(samplerTask);
}

static void samplerLoop(void *arg) {
  uint32_t seq = 0;
  int64_t lastT = 0;
  bool dropPending = false;
  int16_t raw1[7], raw2[7];

  // running timing statistics for the current window
  uint32_t n = 0, overruns = 0, drops = 0, readMaxUs = 0;
  double dtSum = 0, dtSqSum = 0, dtMin = 1e9, dtMax = 0, skewSum = 0;

  for (;;) {
    // Sleep until the timer wakes us. Returns how many ticks arrived; >1 = we missed some.
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Frame f = {};
    f.sync = FRAME_SYNC;
    f.seq = seq++;
    if (pending > 1) { f.flags |= FLAG_OVERRUN; overruns += pending - 1; }
    if (dropPending) { f.flags |= FLAG_DROPPED; dropPending = false; }

    // Timestamps go right before each read, so t2 - t1 is the inter-IMU skew
    int64_t t1 = esp_timer_get_time();
    bool ok1 = readImuRaw(IMU_ADDR[0], raw1);
    int64_t t2 = esp_timer_get_time();
    bool ok2 = readImuRaw(IMU_ADDR[1], raw2);
    int64_t t3 = esp_timer_get_time();

    f.t_us = t1;
    int64_t skew = t2 - t1;
    f.skew_us = skew > 65535 ? 65535 : (uint16_t)skew;

    // raw order: ax ay az temp gx gy gz -> frame order: ax ay az gx gy gz
    if (ok1) {
      for (int k = 0; k < 3; k++) { f.imu1[k] = raw1[k]; f.imu1[k + 3] = raw1[k + 4]; }
      f.temp1 = raw1[3];
    } else {
      f.flags |= FLAG_IMU1_ERR;
    }
    if (ok2) {
      for (int k = 0; k < 3; k++) { f.imu2[k] = raw2[k]; f.imu2[k + 3] = raw2[k + 4]; }
    } else {
      f.flags |= FLAG_IMU2_ERR;
    }

    f.crc = crc16((const uint8_t *)&f, sizeof(Frame) - 2);

    // Non-blocking push: if the writer can't keep up, drop and flag it instead of stalling sampling
    if (xQueueSend(frameQueue, &f, 0) != pdTRUE) { dropPending = true; drops++; }

    // ----- timing statistics -----
    if (lastT != 0) {
      double dt = (double)(t1 - lastT);
      dtSum   += dt;
      dtSqSum += dt * dt;
      if (dt < dtMin) dtMin = dt;
      if (dt > dtMax) dtMax = dt;
      skewSum += (double)skew;
      n++;
    }
    lastT = t1;
    if ((uint32_t)(t3 - t1) > readMaxUs) readMaxUs = (uint32_t)(t3 - t1);

    if (n >= STATS_EVERY && !statsReady) {
      double mean = dtSum / n;
      statsSnap.n         = n;
      statsSnap.dtMean    = mean;
      statsSnap.dtMin     = dtMin;
      statsSnap.dtMax     = dtMax;
      statsSnap.jitter    = sqrt(fmax(dtSqSum / n - mean * mean, 0.0)); // std dev of dt
      statsSnap.skewMean  = skewSum / n;
      statsSnap.readMaxUs = readMaxUs;
      statsSnap.overruns  = overruns;
      statsSnap.drops     = drops;
      statsReady = true;

      // reset for the next window (min starts huge so the first dt replaces it)
      n = 0; overruns = 0; drops = 0; readMaxUs = 0;
      dtSum = dtSqSum = skewSum = 0;
      dtMin = 1e9; dtMax = 0;
    }
  }
}

// Low priority: drains the ring buffer to Serial as CSV, plus a stats line once per second
static void writerLoop(void *arg) {
  Frame f;
  for (;;) {
    if (xQueueReceive(frameQueue, &f, portMAX_DELAY) != pdTRUE) continue;

    Serial.printf("%lu,%lld,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%04X\n",
                  (unsigned long)f.seq, (long long)f.t_us, (unsigned)f.skew_us,
                  f.imu1[0], f.imu1[1], f.imu1[2], f.imu1[3], f.imu1[4], f.imu1[5],
                  f.imu2[0], f.imu2[1], f.imu2[2], f.imu2[3], f.imu2[4], f.imu2[5],
                  f.temp1, (unsigned)f.flags, (unsigned)f.crc);

    if (statsReady) {
      Serial.printf("# STATS n=%lu dt_mean=%.1fus min=%.0f max=%.0f jitter=%.1fus "
                    "skew=%.0fus read_max=%luus overruns=%lu dropped=%lu\n",
                    (unsigned long)statsSnap.n, statsSnap.dtMean, statsSnap.dtMin, statsSnap.dtMax,
                    statsSnap.jitter, statsSnap.skewMean, (unsigned long)statsSnap.readMaxUs,
                    (unsigned long)statsSnap.overruns, (unsigned long)statsSnap.drops);
      statsReady = false;
    }
  }
}

// Create the ring buffer, both tasks, then start the timer (task must exist before the first tick)
inline void startSampler() {
  Serial.println("seq,t_us,skew_us,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,temp1,flags,crc");

  frameQueue = xQueueCreate(QUEUE_DEPTH, sizeof(Frame));
  xTaskCreatePinnedToCore(writerLoop,  "writer",  4096, nullptr, 1, nullptr,      0); // low priority, core 0
  xTaskCreatePinnedToCore(samplerLoop, "sampler", 4096, nullptr, 5, &samplerTask, 1); // high priority, core 1

  esp_timer_create_args_t args = {};
  args.callback = &onSampleTimer;
  args.name = "sample";
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, SAMPLE_PERIOD_US);
}
