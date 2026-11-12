#include <Arduino.h>
#include <Wire.h>
#include <SelfTest.h>
#include <ImuConfig.h>
#include <Timing.h>

void setup() {
  Serial.begin(921600);                       // 200 Hz CSV needs ~18 kB/s; 115200 only carries ~11.5 kB/s
  delay(200);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

  bool ok = selfTest();                       // 1. bus + identity
  ok &= configureImus();                      // 2. reset, wake, configure, verify
  Serial.println(ok ? "# SELFTEST PASS" : "# SELFTEST FAIL");

  startSampler();                             // 3. timer + sampler + writer tasks
}

void loop() { vTaskDelay(portMAX_DELAY); }    // all work happens in the tasks