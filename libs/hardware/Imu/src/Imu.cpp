#include "Imu.h"

#include <BoardConfig.h>

#if FREEINK_CAP_IMU

#include <Wire.h>
#include <soc/soc_caps.h>

#include <cmath>
#include <cstring>

namespace freeink {
namespace {

// LSM6DS3TR-C register map (datasheet).
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t WHO_AM_I_VALUE = 0x6A;  // shared by LSM6DS3 / LSM6DS3TR-C
constexpr uint8_t REG_CTRL1_XL = 0x10;    // accel: ODR + full scale
constexpr uint8_t REG_CTRL2_G = 0x11;     // gyro: ODR + full scale
constexpr uint8_t REG_CTRL3_C = 0x12;     // BDU / auto-increment
constexpr uint8_t REG_OUTX_L_G = 0x22;    // gyro X..Z (6 bytes, LE)
constexpr uint8_t REG_OUTX_L_XL = 0x28;   // accel X..Z (6 bytes, LE)

// 104 Hz (ODR = 0100b in bits [7:4]); accel FS = ±2 g, gyro FS = ±245 dps (00b).
constexpr uint8_t CTRL1_XL_104HZ_2G = 0x40;
constexpr uint8_t CTRL2_G_104HZ_245DPS = 0x40;
constexpr uint8_t CTRL3_C_BDU_IF_INC = 0x44;  // BDU=1, IF_INC=1 (block update + auto-increment)

// Sensitivities for the scales above (datasheet "mechanical characteristics").
constexpr float ACCEL_G_PER_LSB = 0.061f / 1000.0f;  // 0.061 mg/LSB at ±2 g
constexpr float GYRO_DPS_PER_LSB = 8.75f / 1000.0f;  // 8.75 mdps/LSB at ±245 dps

// QMI8658 register map.
constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;
constexpr uint8_t QMI8658_WHO_AM_I_VALUE = 0x05;
constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;
constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;
constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;
constexpr uint8_t QMI8658_REG_CTRL5 = 0x06;
constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;
constexpr uint8_t QMI8658_REG_CTRL8 = 0x09;
constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A;
constexpr uint8_t QMI8658_REG_CAL1_L = 0x0B;  // CAL1_L..CAL4_H are 0x0B..0x12
constexpr uint8_t QMI8658_REG_STATUSINT = 0x2D;
constexpr uint8_t QMI8658_REG_TAP_STATUS = 0x59;
constexpr uint8_t QMI8658_REG_AX_L = 0x35;
constexpr uint8_t QMI8658_REG_GX_L = 0x3B;
constexpr uint8_t QMI8658_ADDR_6A = 0x6A;
constexpr uint8_t QMI8658_ADDR_6B = 0x6B;
// CTRL1 bit5 selects the byte order of the output registers: 0 = little
// endian (the part's reset default), 1 = big endian. read() decodes little
// endian, so this bit must stay CLEAR — it used to be set, which made every
// sample a byte-swapped number. Kept as a named constant so the intent is
// visible rather than an absent bit.
constexpr uint8_t QMI8658_CTRL1_BIG_ENDIAN = 1U << 5;
constexpr uint8_t QMI8658_CTRL1_AUTO_INC = 1U << 6;
constexpr uint8_t QMI8658_CTRL1_SENSOR_DISABLE = 1U << 0;
constexpr uint8_t QMI8658_CTRL1_BASE = QMI8658_CTRL1_AUTO_INC;
constexpr uint8_t QMI8658_CTRL2_FS_2G = 0U << 4;
constexpr uint8_t QMI8658_CTRL2_ODR_28HZ = 0x08;
constexpr uint8_t QMI8658_CTRL3_FS_512DPS = 0b101U << 4;
constexpr uint8_t QMI8658_CTRL3_ODR_28HZ = 0x08;
constexpr uint8_t QMI8658_CTRL7_ACC_GYRO_ENABLE = 0x03;
constexpr uint8_t QMI8658_CTRL7_ACC_ONLY = 0x01;
constexpr uint8_t QMI8658_CTRL7_DISABLE_ALL = 0x00;
// CTRL8 bit7: use STATUSINT.bit7 as the CTRL9 handshake instead of the INT1
// pin. On this board INT1 is wired to the audio amplifier enable, so the pin
// is not available and this bit is mandatory before any CTRL9 command.
constexpr uint8_t QMI8658_CTRL8_HANDSHAKE_STATUSINT = 0x80;
constexpr uint8_t QMI8658_CTRL8_TAP_EN = 0x01;
constexpr uint8_t QMI8658_CTRL9_CMD_ACK = 0x00;
constexpr uint8_t QMI8658_CTRL9_CMD_CONFIGURE_TAP = 0x0C;
constexpr uint8_t QMI8658_STATUSINT_CMD_DONE = 0x80;
// LSM6DS3: ODR bits [7:4] = 0000b powers the sensor down; full-scale bits are
// retained, so restoring the configured CTRL value resumes sampling.
constexpr uint8_t CTRL_ODR_POWER_DOWN = 0x00;
constexpr float QMI8658_ACCEL_G_PER_LSB = 1.0f / 16384.0f;  // ±2 g
constexpr float QMI8658_GYRO_DPS_PER_LSB = 1.0f / 64.0f;    // ±512 dps

bool g_wireReady[2] = {false, false};
TwoWire& sensorWire() {
  const auto& s = BoardConfig::ACTIVE.sensors;
#if SOC_I2C_NUM > 1
  return s.i2cBus == 1 ? Wire1 : Wire;
#else
  return Wire;
#endif
}

void ensureWire() {
  const auto& s = BoardConfig::ACTIVE.sensors;
  const uint8_t bus =
#if SOC_I2C_NUM > 1
      s.i2cBus == 1 ? 1 : 0;
#else
      0;
#endif
  if (g_wireReady[bus]) return;
  if (s.i2cSda < 0 || s.i2cScl < 0) return;  // no sensor bus on this board
  auto& wire = sensorWire();
  wire.begin(s.i2cSda, s.i2cScl, s.i2cHz);
  g_wireReady[bus] = true;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* dst, uint8_t len) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom(addr, len, static_cast<uint8_t>(true)) < len) return false;
  for (uint8_t i = 0; i < len; ++i) dst[i] = wire.read();
  return true;
}

bool qmi8658PresentAt(uint8_t addr) {
  uint8_t who = 0;
  return readRegs(addr, QMI8658_REG_WHO_AM_I, &who, 1) && who == QMI8658_WHO_AM_I_VALUE;
}

bool powerDownQmi8658(uint8_t addr) {
  // Do not short-circuit these writes: even if disabling the sensor engines
  // fails, still try to stop the internal oscillator. This is also used as the
  // cleanup path after a partially failed begin().
  const bool sensorsDisabled = writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL);
  const bool oscillatorDisabled = writeReg(addr, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE | QMI8658_CTRL1_SENSOR_DISABLE);
  return sensorsDisabled && oscillatorDisabled;
}

}  // namespace

bool Imu::begin() {
  begun_ = false;
  addr_ = 0;

  const auto& s = BoardConfig::ACTIVE.sensors;
  const uint8_t configuredAddr = s.imuAddr;
  if (configuredAddr == 0) return false;
  if (s.i2cSda < 0 || s.i2cScl < 0 || s.i2cHz == 0) return false;
  ensureWire();
  uint8_t who = 0;
  switch (s.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      if (!readRegs(configuredAddr, REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VALUE) return false;
      if (!writeReg(configuredAddr, REG_CTRL3_C, CTRL3_C_BDU_IF_INC)) return false;
      if (!writeReg(configuredAddr, REG_CTRL1_XL, CTRL1_XL_104HZ_2G)) return false;
      if (!writeReg(configuredAddr, REG_CTRL2_G, CTRL2_G_104HZ_245DPS)) return false;
      addr_ = configuredAddr;
      break;
    case BoardConfig::ImuType::Qmi8658: {
      // SA0 selects between 0x6A and 0x6B. X3 production revisions have used
      // both, so treat the profile address as a preference rather than a
      // guarantee. This restores the fallback used by the pre-SDK X3 driver.
      const uint8_t alternateAddr = configuredAddr == QMI8658_ADDR_6A ? QMI8658_ADDR_6B : QMI8658_ADDR_6A;
      if (qmi8658PresentAt(configuredAddr)) {
        addr_ = configuredAddr;
      } else if (qmi8658PresentAt(alternateAddr)) {
        addr_ = alternateAddr;
      } else {
        return false;
      }

      ctrl2_ = QMI8658_CTRL2_FS_2G | QMI8658_CTRL2_ODR_28HZ;
      ctrl3_ = QMI8658_CTRL3_FS_512DPS | QMI8658_CTRL3_ODR_28HZ;
      accelScale_ = QMI8658_ACCEL_G_PER_LSB;
      const bool configured = writeReg(addr_, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL) &&
                              writeReg(addr_, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE) &&
                              writeReg(addr_, QMI8658_REG_CTRL2, ctrl2_) &&
                              writeReg(addr_, QMI8658_REG_CTRL3, ctrl3_) &&
                              // Low-pass filters off: the gesture layer does its
                              // own smoothing and the tap engine wants the raw
                              // peaks.
                              writeReg(addr_, QMI8658_REG_CTRL5, 0x00) &&
                              // Handshake over STATUSINT, INT1 being unavailable
                              // on boards that share that pin (see CTRL8 above).
                              writeReg(addr_, QMI8658_REG_CTRL8, QMI8658_CTRL8_HANDSHAKE_STATUSINT) &&
                              writeReg(addr_, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
      mode_ = configured ? Mode::AccelGyro : Mode::Off;
      if (!configured) {
        // A failed setup must not strand a previously running sensor in its
        // multi-milliamp active mode. The digital interface remains available
        // in QMI8658 power-down, so this cleanup is safe to attempt here.
        powerDownQmi8658(addr_);
        return false;
      }
      break;
    }
    case BoardConfig::ImuType::None:
      return false;
  }
  begun_ = true;

  // Sanity check: standing still, the magnitude of the acceleration vector is
  // one g. Anything far from that means the numbers are not acceleration (wrong
  // byte order, wrong scale, a part that answers but is not configured), and a
  // consumer must not offer gestures as if they worked.
  //
  // The wait is the whole point, and a single delay(20) was not enough. The
  // config above leaves the accelerometer at 28 Hz, so one sample takes ~36 ms:
  // reading after 20 ms returns whatever the output registers still hold. On a
  // WARM restart — which is the common case, not the rare one — those registers
  // hold data from the previous session's range (a consumer that picked +/-8 g
  // leaves it there; the part is not reset by a firmware restart), and reading
  // +/-8 g counts through the +/-2 g scale set here divides the magnitude by
  // four: ~0.25 g, comfortably outside the window, so a perfectly healthy part
  // failed its own sanity check and every gesture built on selfCheckPassed()
  // stayed off for the whole session.
  //
  // So: sample until the numbers are fresh, a few periods at a time, and take
  // the first plausible one. Worst case this costs ~200 ms once, at begin().
  selfCheckOk_ = false;
  constexpr int SELF_CHECK_TRIES = 5;
  constexpr uint32_t SAMPLE_PERIOD_MS = 40;  // one period at 28 Hz, rounded up
  for (int attempt = 0; attempt < SELF_CHECK_TRIES && !selfCheckOk_; ++attempt) {
    delay(SAMPLE_PERIOD_MS);
    Sample probe = {};
    if (!read(probe)) continue;
    const float magnitude = sqrtf(probe.ax * probe.ax + probe.ay * probe.ay + probe.az * probe.az);
    selfCheckOk_ = magnitude > 0.6f && magnitude < 1.6f;
    lastSelfCheckG_ = magnitude;
  }
  return true;
}

bool Imu::read(Sample& out) {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  const auto& s = BoardConfig::ACTIVE.sensors;
  uint8_t g[6] = {};
  uint8_t a[6] = {};
  if (s.imuType == BoardConfig::ImuType::Lsm6ds3) {
    if (!readRegs(addr, REG_OUTX_L_G, g, sizeof(g))) return false;
    if (!readRegs(addr, REG_OUTX_L_XL, a, sizeof(a))) return false;
  } else if (s.imuType == BoardConfig::ImuType::Qmi8658) {
    // One burst for both halves (CTRL1 auto-increment): AX..GZ are contiguous
    // from 0x35, so this is a single bus transaction instead of two, and the
    // six values come from the same sample instant.
    uint8_t frame[12] = {};
    if (!readRegs(addr, QMI8658_REG_AX_L, frame, sizeof(frame))) return false;
    memcpy(a, frame, 6);
    memcpy(g, frame + 6, 6);
    if (mode_ == Mode::AccelOnly) memset(g, 0, sizeof(g));
  } else {
    return false;
  }

  const int16_t gx = static_cast<int16_t>(g[0] | g[1] << 8);
  const int16_t gy = static_cast<int16_t>(g[2] | g[3] << 8);
  const int16_t gz = static_cast<int16_t>(g[4] | g[5] << 8);
  const int16_t ax = static_cast<int16_t>(a[0] | a[1] << 8);
  const int16_t ay = static_cast<int16_t>(a[2] | a[3] << 8);
  const int16_t az = static_cast<int16_t>(a[4] | a[5] << 8);

  const float accelScale = s.imuType == BoardConfig::ImuType::Qmi8658
                               ? (accelScale_ > 0.0f ? accelScale_ : QMI8658_ACCEL_G_PER_LSB)
                               : ACCEL_G_PER_LSB;
  const float gyroScale = s.imuType == BoardConfig::ImuType::Qmi8658 ? QMI8658_GYRO_DPS_PER_LSB : GYRO_DPS_PER_LSB;
  out.ax = ax * accelScale;
  out.ay = ay * accelScale;
  out.az = az * accelScale;
  out.gx = gx * gyroScale;
  out.gy = gy * gyroScale;
  out.gz = gz * gyroScale;
  return true;
}

bool Imu::sleep() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  switch (BoardConfig::ACTIVE.sensors.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      return writeReg(addr, REG_CTRL1_XL, CTRL_ODR_POWER_DOWN) && writeReg(addr, REG_CTRL2_G, CTRL_ODR_POWER_DOWN);
    case BoardConfig::ImuType::Qmi8658:
      // CTRL7 only disables sampling; the internal oscillator keeps running.
      // SensorDisable is required for the QMI8658's full power-down mode.
      mode_ = Mode::Off;
      return powerDownQmi8658(addr);
    case BoardConfig::ImuType::None:
      return false;
  }
  return false;
}

bool Imu::wake() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  switch (BoardConfig::ACTIVE.sensors.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      return writeReg(addr, REG_CTRL1_XL, CTRL1_XL_104HZ_2G) && writeReg(addr, REG_CTRL2_G, CTRL2_G_104HZ_245DPS);
    case BoardConfig::ImuType::Qmi8658: {
      // Re-enable the internal oscillator before restarting the sensors.
      const bool ok = writeReg(addr, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE) &&
                      writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
      if (ok) mode_ = Mode::AccelGyro;
      return ok;
    }
    case BoardConfig::ImuType::None:
      return false;
  }
  return false;
}

// --- QMI8658 extras ---------------------------------------------------------

bool Imu::setMode(const Mode mode) {
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  if (mode == mode_) return true;
  uint8_t ctrl7 = QMI8658_CTRL7_DISABLE_ALL;
  if (mode == Mode::AccelOnly) ctrl7 = QMI8658_CTRL7_ACC_ONLY;
  else if (mode == Mode::AccelGyro) ctrl7 = QMI8658_CTRL7_ACC_GYRO_ENABLE;
  // Leaving Off means the oscillator was stopped by sleep(): restart it first.
  if (mode_ == Mode::Off && !writeReg(addr_, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE)) return false;
  if (!writeReg(addr_, QMI8658_REG_CTRL7, ctrl7)) return false;
  mode_ = mode;
  return true;
}

bool Imu::setAccelConfig(const uint8_t fsBits, const uint8_t odrBits) {
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  const uint8_t ctrl2 = static_cast<uint8_t>((fsBits & 0x07) << 4 | (odrBits & 0x0F));
  if (!writeReg(addr_, QMI8658_REG_CTRL2, ctrl2)) return false;
  ctrl2_ = ctrl2;
  // 32768 counts over the full scale: ±2 g -> 16384/g, and one step of the
  // scale field halves that.
  accelScale_ = 1.0f / (16384.0f / static_cast<float>(1U << (fsBits & 0x07)));
  return true;
}

bool Imu::ctrl9(const uint8_t command) {
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  if (!writeReg(addr_, QMI8658_REG_CTRL9, command)) return false;
  // The part signals completion in STATUSINT.bit7 (CTRL8.bit7 selected that
  // over the INT1 pin in begin()). Commands answer in well under a
  // millisecond; the budget here is generous on purpose, it only costs time
  // when something is wrong.
  uint8_t status = 0;
  bool done = false;
  for (int i = 0; i < 100 && !done; ++i) {
    if (readRegs(addr_, QMI8658_REG_STATUSINT, &status, 1) && (status & QMI8658_STATUSINT_CMD_DONE)) done = true;
    else delay(1);
  }
  if (!done) return false;
  // Acknowledge, which is what clears the bit for the next command.
  if (!writeReg(addr_, QMI8658_REG_CTRL9, QMI8658_CTRL9_CMD_ACK)) return false;
  for (int i = 0; i < 100; ++i) {
    if (readRegs(addr_, QMI8658_REG_STATUSINT, &status, 1) && !(status & QMI8658_STATUSINT_CMD_DONE)) return true;
    delay(1);
  }
  return false;
}

bool Imu::configureTap(const uint8_t priority, const uint8_t peakWindow, const uint16_t tapWindow,
                       const uint16_t dTapWindow, const float alpha, const float gamma, const float peakMagThr,
                       const float udmThr) {
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  // Datasheet 10.3: the parameters may only be loaded with both sensors
  // disabled. The previous mode is restored on the way out.
  const Mode previous = mode_;
  if (!writeReg(addr_, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL)) return false;
  mode_ = Mode::Off;

  // Fixed-point conversions, all from Table 35: alpha and gamma are 7-bit
  // fractions, the two thresholds 10-bit fractions.
  const uint8_t alphaFx = static_cast<uint8_t>(alpha * 128.0f + 0.5f);
  const uint8_t gammaFx = static_cast<uint8_t>(gamma * 128.0f + 0.5f);
  const uint16_t peakFx = static_cast<uint16_t>(peakMagThr * 1024.0f + 0.5f);
  const uint16_t udmFx = static_cast<uint16_t>(udmThr * 1024.0f + 0.5f);

  // CAL1_L..CAL4_H, written as one auto-incrementing block per command set.
  const uint8_t first[8] = {peakWindow,
                            priority,
                            static_cast<uint8_t>(tapWindow & 0xFF),
                            static_cast<uint8_t>(tapWindow >> 8),
                            static_cast<uint8_t>(dTapWindow & 0xFF),
                            static_cast<uint8_t>(dTapWindow >> 8),
                            0x00,
                            0x01};  // CAL4_H = 1: first parameter set
  const uint8_t second[8] = {alphaFx,
                             gammaFx,
                             static_cast<uint8_t>(peakFx & 0xFF),
                             static_cast<uint8_t>(peakFx >> 8),
                             static_cast<uint8_t>(udmFx & 0xFF),
                             static_cast<uint8_t>(udmFx >> 8),
                             0x00,
                             0x02};  // CAL4_H = 2: second parameter set
  bool ok = true;
  for (const uint8_t* set : {first, second}) {
    for (uint8_t i = 0; i < 8 && ok; ++i) ok = writeReg(addr_, static_cast<uint8_t>(QMI8658_REG_CAL1_L + i), set[i]);
    if (ok) ok = ctrl9(QMI8658_CTRL9_CMD_CONFIGURE_TAP);
    if (!ok) break;
  }

  mode_ = Mode::Off;
  setMode(previous);
  return ok;
}

bool Imu::enableTap(const bool on) {
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  uint8_t ctrl8 = 0;
  if (!readRegs(addr_, QMI8658_REG_CTRL8, &ctrl8, 1)) return false;
  ctrl8 |= QMI8658_CTRL8_HANDSHAKE_STATUSINT;  // never give the handshake back to INT1
  if (on) ctrl8 |= QMI8658_CTRL8_TAP_EN;
  else ctrl8 &= static_cast<uint8_t>(~QMI8658_CTRL8_TAP_EN);
  return writeReg(addr_, QMI8658_REG_CTRL8, ctrl8);
}

bool Imu::readTapStatus(uint8_t& status) {
  status = 0;
  if (!begun_ || addr_ == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  return readRegs(addr_, QMI8658_REG_TAP_STATUS, &status, 1);
}

}  // namespace freeink

#else  // FREEINK_CAP_IMU — IMU absent.

namespace freeink {
bool Imu::begin() { return false; }
bool Imu::read(Sample&) { return false; }
bool Imu::sleep() { return false; }
bool Imu::wake() { return false; }
bool Imu::setMode(Mode) { return false; }
bool Imu::setAccelConfig(uint8_t, uint8_t) { return false; }
bool Imu::ctrl9(uint8_t) { return false; }
bool Imu::configureTap(uint8_t, uint8_t, uint16_t, uint16_t, float, float, float, float) { return false; }
bool Imu::enableTap(bool) { return false; }
bool Imu::readTapStatus(uint8_t& status) {
  status = 0;
  return false;
}
}  // namespace freeink

#endif  // FREEINK_CAP_IMU
