#include "T1000aBoard.h"

// QST QMA6100P accelerometer, as Seeed's T1000-A firmware drives it (v2.5):
//  - power on GPIO7 (high = on, 0x0102BB50; low and its pins low when off, 0x0102BB34 and boot 0x01029074..8C);
//  - software I2C, SDA = GPIO8 (direction switched for ACK and reads), SCL = GPIO9 (always driven), 0x01038490..;
//  - address 0x12 (0x24 / 0x25 on the wire), register init sequence 0x01022376 (QST's recommended one).
// Seeed only uses its motion / shock interrupts and never reads the axes: the scale comes from the datasheet
// (14-bit samples, range register 0x0F = 0x04 -> +-8 g, 1024 LSB/g); "acc" on the console prints |a| to check it.

#define ACC_ADDR      0x12
#define ACC_CHIP_ID   0x90
#define ACC_LSB_PER_G 1024.0f

static const uint32_t I2C_HALF_US = 5;

static void sclSet(bool v) { digitalWrite(PIN_ACC_SCL, v ? HIGH : LOW); delayMicroseconds(I2C_HALF_US); }
static bool s_sda_out;   // SDA direction, to skip reconfiguring it on every bit (SDA only changes while SCL is low)
static void sdaOut(bool v) {
  if (!s_sda_out) { pinMode(PIN_ACC_SDA, OUTPUT); s_sda_out = true; }
  digitalWrite(PIN_ACC_SDA, v ? HIGH : LOW);
}
static void sdaRelease() { pinMode(PIN_ACC_SDA, INPUT_PULLUP); s_sda_out = false; }

static void i2cStart() {   // also a repeated start: SCL is low between bytes
  sdaOut(true);
  sclSet(true);
  sdaOut(false);
  delayMicroseconds(I2C_HALF_US);
  sclSet(false);
}

static void i2cStop() {
  sdaOut(false);
  sclSet(true);
  sdaOut(true);
  delayMicroseconds(I2C_HALF_US);
}

static bool i2cWrite(uint8_t b) {   // true = ACK
  for (int i = 7; i >= 0; i--) {
    sdaOut(b >> i & 1);
    sclSet(true);
    sclSet(false);
  }
  sdaRelease();
  sclSet(true);
  bool ack = digitalRead(PIN_ACC_SDA) == LOW;
  sclSet(false);
  return ack;
}

static uint8_t i2cRead(bool ack) {
  uint8_t b = 0;
  sdaRelease();
  for (int i = 0; i < 8; i++) {
    sclSet(true);
    b = b << 1 | (digitalRead(PIN_ACC_SDA) == HIGH);
    sclSet(false);
  }
  sdaOut(!ack);
  sclSet(true);
  sclSet(false);
  return b;
}

static bool accWrite(uint8_t reg, uint8_t val) {
  i2cStart();
  bool ok = i2cWrite(ACC_ADDR << 1) && i2cWrite(reg) && i2cWrite(val);
  i2cStop();
  return ok;
}

static bool accRead(uint8_t reg, uint8_t* buf, int n) {
  i2cStart();
  bool ok = i2cWrite(ACC_ADDR << 1) && i2cWrite(reg);
  if (ok) {
    i2cStart();
    ok = i2cWrite(ACC_ADDR << 1 | 1);
    for (int i = 0; ok && i < n; i++) buf[i] = i2cRead(i < n - 1);
  }
  i2cStop();
  return ok;
}

void T1000aBoard::accelPinsOff() {   // unpowered chip: its I2C pins held low, as Seeed does (no back-powering)
  pinMode(PIN_ACC_SCL, OUTPUT); digitalWrite(PIN_ACC_SCL, LOW);
  pinMode(PIN_ACC_SDA, OUTPUT); digitalWrite(PIN_ACC_SDA, LOW);
  s_sda_out = true;
  pinMode(PIN_ACC_PWR, OUTPUT); digitalWrite(PIN_ACC_PWR, LOW);
}

bool T1000aBoard::readAccel(Accel& a) {
  a.chip_id = 0;
  a.x = a.y = a.z = 0;
  pinMode(PIN_ACC_PWR, OUTPUT); digitalWrite(PIN_ACC_PWR, HIGH);
  pinMode(PIN_ACC_SCL, OUTPUT); digitalWrite(PIN_ACC_SCL, HIGH);
  sdaOut(true);
  delay(10);

  bool ok = accRead(0x00, &a.chip_id, 1) && a.chip_id == ACC_CHIP_ID;
  if (ok) {
    static const uint8_t init[][2] = {   // Seeed / QST sequence, 0x01022376 (0xFF = pause, value in ms)
      {0x36, 0xB6}, {0xFF, 5}, {0x36, 0x00}, {0xFF, 10}, {0x11, 0x80}, {0x11, 0x84}, {0x4A, 0x20},
      {0x56, 0x01}, {0x5F, 0x80}, {0xFF, 2}, {0x5F, 0x00}, {0xFF, 10}, {0x0F, 0x04}, {0x10, 0x00},
    };
    for (auto& s : init) {
      if (s[0] == 0xFF) delay(s[1]);
      else if (!(ok = accWrite(s[0], s[1]))) break;
    }
  }
  uint8_t d[6] = {0};
  if (ok) {   // wait for a fresh sample (new-data flag, bit 0 of each LSB register)
    uint32_t t0 = millis();
    while ((ok = accRead(0x01, d, 6)) && !(d[0] & 1) && millis() - t0 < 200) delay(5);
    ok = ok && (d[0] & 1);
  }
  accelPinsOff();
  if (!ok) return false;
  a.x = (int16_t)(d[1] << 8 | d[0]) >> 2;
  a.y = (int16_t)(d[3] << 8 | d[2]) >> 2;
  a.z = (int16_t)(d[5] << 8 | d[4]) >> 2;
  return true;
}

float T1000aBoard::Accel::g(int16_t raw) { return raw / ACC_LSB_PER_G; }
