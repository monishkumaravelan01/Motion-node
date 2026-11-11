#include <Arduino.h>
#include <Wire.h>
#include <SelfTest.h>
#include <ImuConfig.h>


void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

  bool ok = selfTest();                       // 1. bus + identity
  for (uint16_t addr : IMU_ADDR) {
    if (!initImu(addr)) {                     // 2. reset, wake, configure
      Serial.printf("IMU @0x%02X init FAIL\n", addr);
      ok = false;
    }
    ok &= verifyConfig(addr);                 // 3. confirm the settings stuck
  }
  Serial.println(ok ? "# SELFTEST PASS" : "# SELFTEST FAIL");
}

void loop() {
  ImuSample s1 = {}, s2 = {};
  if (readImu(IMU_ADDR[0], s1)) printSample("imu1", s1);
  if (readImu(IMU_ADDR[1], s2)) printSample("imu2", s2);
  delay(500);
}