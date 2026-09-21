#include "PowerManager.h"

#include <Arduino.h>
#include <BoardConfig.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_sleep.h>
#include <soc/soc_caps.h>
#if SOC_PM_SUPPORT_EXT1_WAKEUP
#include <driver/rtc_io.h>
#endif

namespace freeink {
namespace {
bool powerActiveHigh() { return BoardConfig::ACTIVE.input.powerActiveHigh; }
}  // namespace

int8_t PowerManager::wakeSourcePin() {
  const auto& in = BoardConfig::ACTIVE.input;
  return in.wakePin >= 0 ? in.wakePin : in.power;
}

namespace {
// How long waitForPowerButtonRelease() waits before giving up on a stuck pin.
constexpr uint32_t RELEASE_WAIT_MS = 8000;
// Last-resort wake when no button could be armed. Short enough that the device
// does not look dead, long enough not to become a boot loop.
constexpr uint32_t WAKE_FALLBACK_MS = 5 * 60 * 1000;
}  // namespace

bool PowerManager::armWakeOnPins(uint64_t gpioMask, bool wakeLow) {
  if (gpioMask == 0) return false;
#if SOC_PM_SUPPORT_EXT1_WAKEUP
  // Xtensa (S3/S2, classic ESP32): RTC ext1. Pins must be RTC GPIOs — the IDF
  // rejects the whole mask otherwise (ESP_ERR_INVALID_ARG) and nothing is armed,
  // so check up front and name the offending pin. (S3: GPIO0-21 are RTC.)
  for (int pin = 0; pin < 64; ++pin) {
    if (!(gpioMask & (1ULL << pin))) continue;
    if (pin >= GPIO_NUM_MAX || !rtc_gpio_is_valid_gpio(static_cast<gpio_num_t>(pin))) {
      log_e("wake pin GPIO%d is not an RTC GPIO: deep-sleep wake NOT armed", pin);
      return false;
    }
  }
  //
  // The classic ESP32 RTC has no "any low" mode — only ESP_EXT1_WAKEUP_ALL_LOW
  // ("wake when ALL selected pins are low"). For a single wake pin (the common
  // power-button case) ALL_LOW and ANY_LOW are identical; a multi-pin low wake on
  // classic ESP32 fires only when every pin is low. S2/S3 expose ANY_LOW directly.
#if defined(CONFIG_IDF_TARGET_ESP32)
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ALL_LOW;
#else
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ANY_LOW;
#endif
  const esp_err_t err = esp_sleep_enable_ext1_wakeup(gpioMask, wakeLow ? lowMode : ESP_EXT1_WAKEUP_ANY_HIGH);
#elif SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
  // RISC-V (C3/C6/H2): the deep-sleep "gpio" wakeup source.
  const esp_err_t err =
      esp_deep_sleep_enable_gpio_wakeup(gpioMask, wakeLow ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
#error "FreeInk PowerManager: target has no supported deep-sleep GPIO wakeup source"
#endif
  if (err != ESP_OK) {
    log_e("deep-sleep GPIO wake NOT armed (mask 0x%08lx%08lx): %s", static_cast<unsigned long>(gpioMask >> 32),
          static_cast<unsigned long>(gpioMask), esp_err_to_name(err));
    return false;
  }
  return true;
}

bool PowerManager::armPowerButtonWakeup() {
  const int8_t pin = wakeSourcePin();
  if (pin < 0) return false;
  const bool activeHigh = powerActiveHigh();

  // Hold the idle level with the opposite pull so the line is defined in sleep.
  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  if (!armWakeOnPins(1ULL << pin, /*wakeLow=*/!activeHigh)) {
    log_e("power/wake button on GPIO%d could not be armed as a deep-sleep wake source", pin);
    return false;
  }
  return true;
}

void PowerManager::waitForPowerButtonRelease() {
  const int8_t pin = wakeSourcePin();
  if (pin < 0) return;
  const bool activeHigh = powerActiveHigh();

  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  const int pressedLevel = activeHigh ? HIGH : LOW;
  // Bounded wait. A pin stuck at the pressed level (a jammed button, a shorted
  // line, a pull that lost its rail) used to hang the whole shutdown here, in a
  // delay(50) loop with no way out: the device never reached sleep and never
  // came back to the UI either. Giving up after the timeout is strictly better
  // than hanging — the worst case is that the device wakes immediately, which
  // the user can see and act on, instead of looking dead.
  const uint32_t startedAt = millis();
  while (digitalRead(pin) == pressedLevel) {
    if (millis() - startedAt >= RELEASE_WAIT_MS) {
      log_e("wake pin GPIO%d is still at the pressed level after %u ms: sleeping anyway", pin,
            static_cast<unsigned>(RELEASE_WAIT_MS));
      return;
    }
    delay(50);
  }
}

namespace {
// Drive a rail-enable pin to `offLevel` and latch it so the level survives deep
// sleep (requires gpio_deep_sleep_hold_en(), done in deepSleep()). gpio_hold_dis
// first: a hold left over from a previous cycle would make the writes no-ops.
void holdRailOff(int8_t pin, uint8_t offLevel) {
  if (pin < 0) return;
  const auto g = static_cast<gpio_num_t>(pin);
  gpio_hold_dis(g);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offLevel);
  gpio_hold_en(g);
}
}  // namespace

void PowerManager::powerDownRailsForSleep() {
  const auto& b = BoardConfig::ACTIVE;
  // Keep RESET defined through deep sleep, but never drive an unpowered panel's
  // input HIGH: on boards with a gated EPD rail (Sticky), that can back-power the
  // controller through its RESET protection diode and turn sleep into a
  // milliamp-level drain. Hold RESET LOW alongside a switched-off rail. Boards
  // whose panel rail remains powered (X4 Pro) keep RESET HIGH so a UC8179 cannot
  // drift out of DSLP and restart its analog booster. EpdBus and XteinkDetect
  // release the hold before issuing a reset pulse on wake.
  const uint8_t resetSleepLevel = b.display.powerEnable >= 0 ? LOW : HIGH;
  holdRailOff(b.display.rst, resetSleepLevel);
  holdRailOff(b.display.powerEnable, LOW);
  // SD enable OFF = the inactive level: LOW for active-high enables, HIGH for the
  // active-low ones (e.g. X4 Pro's GPIO5, which powers the card while held LOW).
  holdRailOff(b.sd.powerEnable, b.sd.powerActiveHigh ? LOW : HIGH);
  holdRailOff(b.touch.powerEnable, b.touch.powerEnableActiveHigh ? LOW : HIGH);
  // The mic enable also carries a polarity flag; OFF is the inactive level.
  holdRailOff(b.mic.enable, b.mic.enableActiveHigh ? LOW : HIGH);
}

void PowerManager::deepSleep() {
  esp_sleep_config_gpio_isolate();
#if !FREEINK_MCU_C61
  gpio_deep_sleep_hold_en();
#endif
  esp_deep_sleep_start();
  while (true) {
  }  // esp_deep_sleep_start() does not return; satisfy [[noreturn]]
}

void PowerManager::deepSleepUntilPowerButton() {
  waitForPowerButtonRelease();
  // NEVER sleep without a wake source. The return of armPowerButtonWakeup() used
  // to be dropped on the floor: when the IDF refused the mask (a bad profile, a
  // pin that is not an RTC GPIO, a rare failure) the device went to sleep with
  // nothing able to wake it. From the outside that is indistinguishable from a
  // dead device, and the only way back is the board's hardware escape, if it has
  // one.
  //
  // A timer needs no GPIO and cannot be refused for pin reasons, so it is the one
  // source that always remains. It overwrites a timer the caller may have armed
  // for its own reasons (an alarm), and that is the lesser evil: a late alarm
  // beats a device that never wakes.
  if (!armPowerButtonWakeup()) {
    log_e("no button wake armed: falling back to a %u s timer so the device comes back",
          static_cast<unsigned>(WAKE_FALLBACK_MS / 1000));
    // And this return is CHECKED too. Saying "a timer always remains" while
    // dropping the one value that says whether it was armed guarantees nothing:
    // the API returns esp_err_t, so it can refuse, and the call right after is
    // the irreversible one. Deep sleep with no wake source at all is
    // indistinguishable from a dead device, and the only way back is the
    // board's hardware escape, if it has one.
    const esp_err_t err = esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(WAKE_FALLBACK_MS) * 1000ULL);
    if (err != ESP_OK) {
      // A restart is recoverable and sleeping without a wake source is not, so
      // when there is no confirmed source left the device restarts instead. It
      // comes back on its own, the boot log names the reason, and whatever made
      // both arms fail gets a fresh chance.
      log_e("neither the wake button nor the fallback timer could be armed (%s): restarting instead of "
            "sleeping with no way back",
            esp_err_to_name(err));
      esp_restart();
    }
  }
  deepSleep();
}

}  // namespace freeink
