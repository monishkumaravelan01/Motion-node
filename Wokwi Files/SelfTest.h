#pragma once
#include <Arduino.h>
#include <Wire.h>

// ----- Self Test constants -----
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint32_t I2C_FREQ_HZ = 400000;   // 400 kHz fast mode from IMU data sheet
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint16_t IMU_ADDR[2] = {0x68, 0x69};


// Scan the bus and find all the devices connected to the MCU
int scanBus() {
  Serial.println("# I2C scan");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { // if the device ack then it returns 0
      Serial.printf("found device at 0x%02X\n", a);
      found++;
    }
  }
  Serial.printf("%d device(s)\n", found);
  return found;
}

bool readRegs(uint16_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if(Wire.requestFrom(addr, (size_t)n, true) != n) return false;
  for(uint8_t i = 0; i<n; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// Bus + identity checks. Returns true only if everything passed.
bool selfTest() {
  bool ok = true;

  int deviceCount = scanBus();
  if (deviceCount != 2) {
    Serial.printf("%d device(s) -> FAIL (expected 2, check AD0 wiring)\n", deviceCount);
    ok = false;
  }

  for (uint16_t addr : IMU_ADDR) {
    uint8_t who = 0;
    if (!readRegs(addr, REG_WHO_AM_I, &who, 1)) {
      Serial.printf("IMU @0x%02X no ACK -> FAIL\n", addr);
      ok = false;
    } else if (who != 0x68) {
      Serial.printf("IMU @0x%02X WHO_AM_I = 0x%02X UNEXPECTED -> FAIL\n", addr, who);
      ok = false;
    } else {
      Serial.printf("IMU @0x%02X WHO_AM_I = 0x%02X OK\n", addr, who);
    }
  }
  return ok;
}
