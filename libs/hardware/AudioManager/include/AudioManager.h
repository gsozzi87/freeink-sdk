#pragma once

// FreeInk audio output.
//
// Drives the audio path described by BoardConfig::ACTIVE.audio: bring up the
// control codec over its (possibly shared) I2C bus, master the I2S bus with
// the new ESP-IDF i2s_std driver, and stream 16-bit PCM WAV data from a
// caller-supplied byte source. Two codecs are supported, selected by
// AudioConfig::output: ES8388 (Murphy M3, OEM-recovered register sequence)
// and ES8311 (M5 PaperColor, mirroring M5Unified's speaker bring-up — the
// codec clocks itself from BCLK, plus the AW8737A amp on its ampEnable pin;
// boards with an MCLK line, like the Waveshare ws397, get the vendor MCLK-fed
// bring-up instead).
//
// Capture: when the board wires the codec's ADC output back to the SoC
// (AudioConfig::din, ES8311 only), beginCapture()/readCapture() pull 16-bit
// mono PCM from the analog mic through the same I2S port (full duplex: RX
// borrows TX's BCLK/WS, so playback and capture share one sample rate).
// setMicGain() sube o baja lo que entra por ese micrófono (ver más abajo).
// PDM mics are a separate capability (Microphone).
//
// Playback runs in a dedicated FreeRTOS task (priority above typical workers,
// like the OEM "musicTask"), so play() returns immediately; with loop=true the
// source is rewound and replayed until stop() is called — the alarm use case.
//
// The WAV source is a pair of callbacks instead of a FILE/Stream so the SDK
// stays storage-agnostic: firmware can serve bytes from LittleFS, SD, or a
// PROGMEM array with the same API.

#include <Arduino.h>

#include <functional>

namespace freeink {

class AudioManager {
 public:
  struct WavSource {
    // Copy up to len bytes to dst, returning the count (0 = EOF, <0 = error).
    std::function<int(uint8_t* dst, size_t len)> read;
    // Absolute seek from the start of the WAV; used for chunk walking and loop
    // rewind. Return false if unsupported (loop and header re-parse then fail).
    std::function<bool(size_t pos)> seek;
  };

  // Initializes the codec + enable pin. Returns false when the active board
  // has no audio path (callers can treat audio as absent).
  bool begin();
  bool present() const;

  // Analog output volume, 0-100 (maps onto the codec's OUT1/OUT2 registers).
  void setVolume(uint8_t percent);

  // Starts WAV playback (16-bit PCM, mono or stereo, 8-48 kHz). Stops any
  // current playback first. loop=true replays until stop().
  bool play(const WavSource& source, bool loop);

  // Convenience: play from a memory buffer (e.g. an embedded default sound).
  bool playBuffer(const uint8_t* data, size_t len, bool loop);

  void stop();
  bool isPlaying() const { return playing_; }

  // Codec power-down (CHIPPOWER off). begin() restores it.
  void powerDown();

  // Full release: stop playback and capture, power the codec down and delete
  // the I2S channels, so a later begin()/play() (or another instance) can
  // re-create the port. powerDown() alone keeps the channels allocated.
  void end();

  // --- Capture through the output codec (ES8311 ADC, AudioConfig::din) ---
  // True when the active board routes an analog mic through the codec.
  bool captureAvailable() const;
  // Powers the codec ADC and starts the I2S RX channel at sampleRate
  // (8-48 kHz). Stops any playback first: the port has one clock.
  bool beginCapture(uint32_t sampleRate);
  // Reads up to maxSamples 16-bit mono samples, blocking up to timeoutMs for
  // data. Returns samples read (0 = timeout, <0 = not capturing / error).
  int readCapture(int16_t* dst, size_t maxSamples, uint32_t timeoutMs = 100);
  // Stops the RX channel and powers the codec ADC down. Playback keeps working.
  void endCapture();

  // --- Ganancia del micrófono (ES8311) ---------------------------------------
  // Cuánto se le sube al micrófono, 0-100 %. El PGA ANALÓGICO ya queda siempre
  // al máximo (30 dB, reg 0x14 = 0x1A), así que lo que mueve el porcentaje es
  // la ganancia DIGITAL del ADC por encima de la que deja el vendor:
  // 0 % = tal cual venía y 100 % = +MIC_GAIN_MAX_DB dB. Pasarse satura y le
  // recorta las puntas a la voz, que para el dictado es peor que quedarse corto.
  //
  // Hay un solo códec por placa, así que el ajuste es estático: lo comparten
  // todas las instancias (grabadora de voz, prueba de audio) y se aplica en
  // cada beginCapture(), además de en caliente si el micrófono ya está abierto.
  static constexpr int MIC_GAIN_MAX_DB = 36;
  static constexpr uint8_t MIC_GAIN_DEFAULT = 50;  // +18 dB sobre el vendor
  static void setMicGain(uint8_t percent);
  static uint8_t micGain() { return s_micGain; }
  // Decibeles que agrega el ajuste actual, para poder mostrarlos.
  static int micGainDb() { return static_cast<int>(s_micGain) * MIC_GAIN_MAX_DB / 100; }
  bool isCapturing() const { return capturing_; }
  uint32_t captureSampleRate() const { return capturing_ ? currentRate_ : 0; }

 private:
  struct WavInfo {
    uint32_t sampleRate = 0;
    uint16_t channels = 0;
    uint16_t bitsPerSample = 0;
    size_t dataStart = 0;
    size_t dataLength = 0;
  };

  static void taskEntry(void* self);
  void taskLoop();
  bool parseWavHeader(const WavSource& source, WavInfo& info);
  bool ensureI2s(uint32_t sampleRate);
  void teardownI2s();

  bool codecInit();
  // No toca estado de la instancia (el códec es uno solo y sale de BoardConfig),
  // así que es estática: la ganancia del micrófono también la necesita.
  static bool codecWrite(uint8_t reg, uint8_t value);
  static void applyMicGain();  // ES8311: 0x16 ADC_SCALE + 0x17 ADC_VOLUME
  void codecMute(bool mute);
  void codecCapture(bool on);  // ES8311 ADC / PGA power and mic routing
  void setAmp(bool on);

  bool begun_ = false;
  volatile bool playing_ = false;
  volatile bool stopRequested_ = false;
  TaskHandle_t task_ = nullptr;

  WavSource source_;
  WavInfo wav_;
  bool loop_ = false;

  void* txChan_ = nullptr;  // i2s_chan_handle_t (kept void* to slim the header)
  volatile bool chanEnabled_ = false;
  uint32_t currentRate_ = 0;

  // RX side of the same port, created with TX when the board has a codec mic.
  void* rxChan_ = nullptr;
  volatile bool rxEnabled_ = false;
  volatile bool capturing_ = false;

  static uint8_t s_micGain;
};

}  // namespace freeink

using AudioManager = freeink::AudioManager;
