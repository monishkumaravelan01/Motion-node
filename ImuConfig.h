#pragma once
#include <Wire.h>
#include <SelfTest.h>

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
constexpr uint8_t VAL_GYRO_1000  = 0x10;  // ±1000 dps
constexpr uint8_t VAL_ACCEL_8G   = 0x10;  // ±8 g

// ---- Scale factors: must match the ranges above ----
constexpr float ACCEL_LSB_PER_G  = 4096.0f;
constexpr float GYRO_LSB_PER_DPS = 32.8f;

struct ImuSample {
  float a[3]; //unit g
  float g[3]; //unit deg/s
  float tempC;
};

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
  writeStatus &= writeReg(addr, REG_GYRO_CONFIG, VAL_GYRO_1000);
  writeStatus &= writeReg(addr, REG_ACCEL_CONFIG, VAL_ACCEL_8G);
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
  const uint8_t expected[4] = {VAL_DIV_200HZ,  VAL_DLPF_44HZ, VAL_GYRO_1000, VAL_ACCEL_8G};

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

// Combine two bytes (high byte first) into a signed 16-bit value
static inline int16_t be16(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]);
}