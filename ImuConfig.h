#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "SelfTest.h"   // readRegs(), IMU_ADDR

// ----- IMU config and readings in real unit -----
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;

// ---- Values we write, and what they mean ----
constexpr uint8_t VAL_RESET      = 0x80;  // bit 7: device reset
constexpr uint8_t VAL_WAKE_PLL   = 0x01;  // sleep = 0, clock = gyro X PLL
constexpr uint8_t VAL_DLPF_44HZ  = 0x03;  // accel 44 Hz, gyro 42 Hz
constexpr uint8_t VAL_DIV_200HZ  = 4;     // 1 kHz / (1 + 4) = 200 Hz
constexpr uint8_t VAL_GYRO_RANGE  = 0x10; // ±1000 dps (bits 4:3 = 10)
constexpr uint8_t VAL_ACCEL_RANGE = 0x10; // ±8 g (bits 4:3 = 10)

// ---- Scale factors: derived from the range bits, single source of truth ----
// Change VAL_ACCEL_RANGE / VAL_GYRO_RANGE and the divisor follows automatically (test 4 fix)
constexpr float ACCEL_LSB_TABLE[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f}; // ±2, ±4, ±8, ±16 g
constexpr float GYRO_LSB_TABLE[4]  = {131.0f, 65.5f, 32.8f, 16.4f};         // ±250, ±500, ±1000, ±2000 dps
constexpr float ACCEL_LSB_PER_G    = ACCEL_LSB_TABLE[(VAL_ACCEL_RANGE >> 3) & 0x03];
constexpr float GYRO_LSB_PER_DPS   = GYRO_LSB_TABLE[(VAL_GYRO_RANGE >> 3) & 0x03];

struct ImuSample {
  float a[3]; //unit g
  float g[3]; //unit deg/s
  float tempC;
};

// Combine two bytes (high byte first) into a signed 16-bit value
// (must be defined before readImu uses it)
static inline int16_t be16(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

bool writeReg(uint16_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); // prepares the device in that address
  Wire.write(reg); // prepares the register for write
  Wire.write(val); // writes the value in that specific reg
  return Wire.endTransmission() == 0;
}


bool initImu(uint16_t addr) {
  bool writeStatus = true;
  writeStatus &= writeReg(addr, REG_PWR_MGMT_1, VAL_RESET);
  delay(100);
  writeStatus &= writeReg(addr, REG_PWR_MGMT_1, VAL_WAKE_PLL);
  writeStatus &= writeReg(addr, REG_CONFIG, VAL_DLPF_44HZ);
  writeStatus &= writeReg(addr, REG_GYRO_CONFIG, VAL_GYRO_RANGE);
  writeStatus &= writeReg(addr, REG_ACCEL_CONFIG, VAL_ACCEL_RANGE);
  writeStatus &= writeReg(addr, REG_SMPLRT_DIV, VAL_DIV_200HZ);
  return writeStatus;
}

bool verifyConfig(uint16_t addr) {
  // The four config registers sit next to each other: 0x19, 0x1A, 0x1B, 0x1C.
  // So we can read all four in one burst, starting at 0x19.
  uint8_t cfg[4];
  if (!readRegs(addr, REG_SMPLRT_DIV, cfg, 4)) {
    Serial.printf("IMU @0x%02X config read-back failed (no ACK)\n", addr);
    return false;
  }

  // What we expect, in the same order the burst returns them
  const uint8_t regs[4]     = {REG_SMPLRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG};
  const uint8_t expected[4] = {VAL_DIV_200HZ,  VAL_DLPF_44HZ, VAL_GYRO_RANGE, VAL_ACCEL_RANGE};

  bool ok = true;
  for (int i = 0; i < 4; i++) {
    if (cfg[i] != expected[i]) {
      Serial.printf("IMU @0x%02X reg 0x%02X = 0x%02X, expected 0x%02X -> FAIL\n",addr, regs[i], cfg[i], expected[i]);
      ok = false;   // latch: keep checking the rest, but the result stays false
    }
  }
  if (ok) Serial.printf("IMU @0x%02X config verified OK\n", addr);
  return ok;
}

// Init + verify both IMUs, so setup() stays one line per stage
bool configureImus() {
  bool ok = true;
  for (uint16_t addr : IMU_ADDR) {
    if (!initImu(addr)) {                     // reset, wake, configure
      Serial.printf("IMU @0x%02X init FAIL\n", addr);
      ok = false;
    }
    ok &= verifyConfig(addr);                 // confirm the settings stuck
  }
  return ok;
}

// Raw counts for the sampler: ax ay az temp gx gy gz (no printing, runs 200x per second)
bool readImuRaw(uint16_t addr, int16_t raw[7]) {
  uint8_t reg[14];
  if (!readRegs(addr, REG_ACCEL_XOUT_H, reg, 14)) return false;
  for (int i = 0; i < 7; i++) {
    raw[i] = be16(&reg[2 * i]);
  }
  return true;
}

bool readImu(uint16_t addr, ImuSample &s) {
  // One 14-byte burst from 0x3B, so all values come from the same sample instant
  uint8_t reg[14];
  if (!readRegs(addr, REG_ACCEL_XOUT_H, reg, 14)) {
    Serial.printf("IMU @0x%02X data read failed (no ACK)\n", addr);
    return false;
  }

  // Byte layout: ax 0-1 | ay 2-3 | az 4-5 | temp 6-7 | gx 8-9 | gy 10-11 | gz 12-13
  for (int k = 0; k < 3; k++) {
    s.a[k] = be16(&reg[2 * k])     / ACCEL_LSB_PER_G;    // accel at 0, 2, 4
    s.g[k] = be16(&reg[8 + 2 * k]) / GYRO_LSB_PER_DPS;   // gyro at 8, 10, 12
  }
  s.tempC = be16(&reg[6]) / 340.0f + 36.53f;              // temp at 6
  return true;
}

void printSample(const char *name, const ImuSample &s) {
  Serial.printf("%s a=[%6.2f %6.2f %6.2f] g  w=[%7.1f %7.1f %7.1f] dps  T=%.1f C\n",name, s.a[0], s.a[1], s.a[2], s.g[0], s.g[1], s.g[2], s.tempC);
}