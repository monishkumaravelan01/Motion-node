#include <Arduino.h>
#include <Wire.h>

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint32_t I2C_FREQ_HZ = 400000;   // 400 kHz fast mode from IMU data sheet
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint16_t IMU_ADDR[2] = {0x68, 0x69};

bool readReg(uint16_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // false = keep the bus, Dont trigger the STOP signal
  if (Wire.requestFrom(addr, (size_t)1, true) != 1) return false;
  val = Wire.read();
  return true;
}

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

void setup() {
  Serial.begin(115200); //  baud rate
  delay(200);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
  int deviceCount = scanBus();
  bool status = true;
   if(deviceCount != 2){
    status = false;
    Serial.printf("Lesser device found, check for AD0 connections\n");
  }
  for (uint16_t addr : IMU_ADDR) {
    uint8_t who = 0;
    if (readReg(addr, REG_WHO_AM_I, who)) {
      Serial.printf("IMU @0x%02X WHO_AM_I = 0x%02X %s\n", addr, who, who == 0x68 ? "OK" : "UNEXPECTED");      
    }
    else {
      status = false;
      Serial.printf("IMU @0x%02X no ACK -> FAIL\n", addr);
    }
  }
  Serial.printf(status ? "#SELFTEST PASS" : "#SELFTEST FAILED");
}

void loop() { delay(1000); }