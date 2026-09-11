#pragma once

// FreeInk inertial measurement unit (LSM6DS3TR-C or QMI8658, 6-axis accel +
// gyro).
//
// Reads acceleration (g) and angular rate (deg/s) from the I2C IMU described by
// BoardConfig::ACTIVE.sensors (imuAddr / sensor bus). Dependency-free Wire
// access, mirroring BatteryMonitor. Boards without an IMU (FREEINK_CAP_IMU off,
// or imuAddr == 0) link stub bodies and present() returns false.

#include <Arduino.h>

#include <cstdint>

namespace freeink {

class Imu {
 public:
  struct Sample {
    float ax, ay, az;  // acceleration, g (1 g ~= 9.81 m/s^2)
    float gx, gy, gz;  // angular rate, degrees/second
  };

  // What the part is sampling. The gyro is by far the expensive half (~750 uA
  // against ~142 uA for the accelerometer alone at 250 Hz), and tilt, shake and
  // face-down only need acceleration — so a consumer that polls all day asks
  // for AccelOnly and turns the gyro on for the few screens that use rotation.
  enum class Mode : uint8_t { Off, AccelOnly, AccelGyro };

  // Verifies WHO_AM_I and configures accel + gyro for the active board.
  // Returns false when the active board has no IMU or the part doesn't identify.
  bool begin();
  bool present() const { return begun_; }

  // Reads one accel + gyro sample. Returns false on I2C error.
  bool read(Sample& out);

  // Puts the sensors into hardware standby / power-down. Config registers are
  // retained, so wake() restores sampling without a full begin(). Returns
  // false when the IMU is absent or on I2C error.
  bool sleep();

  // Restarts sampling after sleep(). Returns false when absent or on I2C
  // error; allow for a settling transient before trusting samples.
  bool wake();

  // --- QMI8658 extras -------------------------------------------------------
  // Everything below is a no-op returning false on other parts.

  // Selects what is sampled (see Mode). begin() leaves the part in AccelGyro
  // for compatibility with the callers that predate this.
  bool setMode(Mode mode);
  Mode mode() const { return mode_; }

  // Accelerometer full scale and output rate, as raw CTRL2 fields:
  //   fsBits 0=±2 g 1=±4 g 2=±8 g 3=±16 g, odrBits 4=500 Hz 5=250 Hz 6=125 Hz.
  // The tap engine wants >= 200 Hz. Also updates the g/LSB used by read().
  bool setAccelConfig(uint8_t fsBits, uint8_t odrBits);

  // One CTRL9 command with its handshake (STATUSINT bit7, then CTRL_CMD_ACK).
  // Returns false if the part never signalled completion.
  bool ctrl9(uint8_t command);

  // Loads the tap engine parameters (two CTRL9 rounds, see datasheet 10.3).
  // Must run with the sensors disabled: it does that itself and restores the
  // previous mode. peakMagThr/udmThr are in g^2 and g.
  bool configureTap(uint8_t priority, uint8_t peakWindow, uint16_t tapWindow, uint16_t dTapWindow, float alpha,
                    float gamma, float peakMagThr, float udmThr);
  // CTRL8.bit0. The engine only runs with the accelerometer enabled.
  bool enableTap(bool on);
  // Raw TAP_STATUS (0x59): bits[1:0] 1 = single, 2 = double; bits[5:4] axis;
  // bit7 direction. Reading it clears the event. 0 when nothing was detected.
  bool readTapStatus(uint8_t& status);

  // False when begin() saw an implausible gravity vector: the part answers on
  // I2C but its numbers cannot be trusted, so a consumer should not present
  // gesture detection as working.
  // La magnitud que midió el autochequeo, en g. Sirve para que el consumidor
  // pueda MOSTRAR por qué falló en vez de decir sólo "no responde": un 0,25
  // grita "escala equivocada" y un 0,00 grita "el chip no contesta".
  float lastSelfCheckG() const { return lastSelfCheckG_; }

  bool selfCheckPassed() const { return selfCheckOk_; }

 private:
  bool begun_ = false;
  Mode mode_ = Mode::Off;
  bool selfCheckOk_ = false;
  float lastSelfCheckG_ = 0.0f;
  uint8_t ctrl2_ = 0;  // last accelerometer CTRL2 written
  uint8_t ctrl3_ = 0;  // last gyroscope CTRL3 written
  float accelScale_ = 0.0f;  // g per LSB for the configured full scale
  // The QMI8658 can legally appear at 0x6A or 0x6B depending on its SA0
  // strap. Keep the address found by begin() instead of repeatedly using the
  // board profile's preferred address.
  uint8_t addr_ = 0;
};

}  // namespace freeink

using Imu = freeink::Imu;
