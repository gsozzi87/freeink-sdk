#include "AudioManager.h"

#include <BoardConfig.h>

// Ganancia del micrófono: un solo códec por placa, un solo valor compartido.
// Vive fuera del #if para que el stub sin audio también enlace.
namespace freeink {
uint8_t AudioManager::s_micGain = AudioManager::MIC_GAIN_DEFAULT;
const char* AudioManager::s_lastCaptureError = nullptr;
AudioManager* AudioManager::s_portOwner = nullptr;
}  // namespace freeink

// Capability-gated like FrontlightManager: devices without FREEINK_CAP_AUDIO
// compile the stub bodies at the bottom and link no I2S/codec code.
#if FREEINK_CAP_AUDIO

#include <Wire.h>
#include <driver/i2s_std.h>

#include <memory>

namespace freeink {

namespace {

constexpr uint32_t CODEC_I2C_HZ = 100000;  // OEM bus speed (shared with touch)
constexpr size_t READ_CHUNK = 1024;        // mono source bytes per loop pass
// Silencio antes y despues de levantar el amplificador, en milisegundos (ver
// taskLoop): antes iba en buffers, que duran la mitad o el triple segun la tasa.
constexpr uint32_t AMP_PRIME_MS = 32;
// 150 ms, no 40. El log del aparato mostro clics de 38 ms reproducidos de punta
// a punta ("sono 0: 1824 muestras ... 185 ms") que NADIE oyo, mientras el pitido
// de 160 ms repetido si se oye y a la voz de las cartas "se le comia la primera
// silaba". Todo lo que dura menos de ~100 ms despues del unmute del DAC y del
// enable del clase D se pierde: el ES8311 sube el volumen con una rampa al
// salir del mute y el amplificador tarda en arrancar. El silencio es gratis.
constexpr uint32_t AMP_SETTLE_MS = 150;

// ES8388 playback init recovered from the Murphy OEM firmware — exact register
// order matters (staged mute -> clocks/format -> mixers -> power -> unmute).
struct RegVal {
  uint8_t reg, val;
};
constexpr RegVal ES8388_INIT[] = {
    {0x19, 0x04}, {0x01, 0x50}, {0x02, 0x00}, {0x08, 0x00}, {0x04, 0x3e}, {0x00, 0x12},
    {0x17, 0x18}, {0x18, 0x02}, {0x26, 0x1b}, {0x27, 0x90}, {0x2a, 0x90}, {0x2b, 0x80},
    {0x2d, 0x00}, {0x1b, 0x00}, {0x1a, 0x00}, {0x03, 0xff}, {0x09, 0x88}, {0x0a, 0xf0},
    {0x0b, 0x80}, {0x0c, 0x0e}, {0x0d, 0x02}, {0x10, 0x20}, {0x11, 0x20}, {0x2e, 0x1e},
    {0x2f, 0x1e}, {0x30, 0x1e}, {0x31, 0x1e}, {0x04, 0x3c}, {0x19, 0x00},
};

constexpr uint8_t ES8388_VOL_MAX_REG = 0x21;  // register full-scale for OUT volumes

// ES8311 playback init, mirroring M5Unified's PaperColor speaker bring-up
// (_speaker_enabled_cb_papercolor). The codec derives its internal MCLK from
// BCLK (reg 0x01 bit7 + reg 0x02 MULT_PRE=3, i.e. 32*fs * 8 = 256*fs), so no
// MCLK line is needed and the same init covers every sample rate. The 16-bit
// I2S SDP format (0x09) is set explicitly — M5Unified relies on the default.
constexpr RegVal ES8311_INIT[] = {
    {0x00, 0x80},  // RESET: CSM power on, slave mode
    {0x01, 0xB5},  // CLK_MANAGER: internal MCLK from BCLK pin, clocks on
    {0x02, 0x18},  // CLK_MANAGER: MULT_PRE x8 -> internal MCLK = 256*fs
    {0x09, 0x0C},  // SDP-in: I2S format, 16-bit
    {0x0D, 0x01},  // SYSTEM: power up analog circuitry
    {0x12, 0x00},  // SYSTEM: power up DAC
    {0x13, 0x10},  // SYSTEM: enable output to HP drive
    {0x32, 0xCF},  // DAC volume +16 dB (M5's default for the 1W speaker)
    {0x37, 0x08},  // DAC: bypass equalizer
};

constexpr uint8_t ES8311_VOL_MAX_REG = 0xCF;  // +16 dB, 0.5 dB/step (0xBF = 0 dB)

// ES8311 bring-up for boards that feed the codec's MCLK pin (AudioConfig::mclk
// wired; the I2S master outputs 256*fs there). Mirrors the vendor es8311_init()
// shipped with the Waveshare ws397 examples (Espressif's es8311 component):
// clocks from MCLK with unity pre-divider/multiplier, ADC and DAC OSR at the
// 256*fs defaults, 16-bit I2S on both serial ports, analog + ADC modulator +
// DAC powered, EQ bypassed. Preceded by the full reset (0x1F -> 0x00 -> 0x80)
// with the vendor's settle delay, done in codecInit(). ADC/mic routing is
// applied on demand by codecCapture().
constexpr RegVal ES8311_INIT_MCLK[] = {
    {0x01, 0x3F},  // CLK_MANAGER: MCLK from pin, every clock (ADC + DAC) on
    {0x02, 0x00},  // CLK_MANAGER: DIV_PRE 1, MULT_PRE x1 -> internal MCLK = 256*fs
    {0x03, 0x10},  // ADC: single-speed, OSR 0x10
    {0x04, 0x10},  // DAC: OSR 0x10
    {0x05, 0x00},  // ADC/DAC clock dividers 1
    {0x09, 0x0C},  // SDP-in (DAC): I2S, 16-bit
    {0x0A, 0x0C},  // SDP-out (ADC): I2S, 16-bit
    {0x0D, 0x01},  // SYSTEM: power up analog circuitry
    {0x0E, 0x02},  // SYSTEM: enable analog PGA + ADC modulator
    {0x12, 0x00},  // SYSTEM: power up DAC
    {0x13, 0x10},  // SYSTEM: enable output to HP drive
    {0x1C, 0x6A},  // ADC: EQ bypass, cancel DC offset digitally
    {0x32, 0xB2},  // DAC volume: vendor default (70 %) for the 1 W speaker
    {0x37, 0x08},  // DAC: bypass equalizer
};
constexpr RegVal ES8311_RESET_SEQ[] = {{0x00, 0x1F}, {0x00, 0x00}, {0x00, 0x80}};

uint32_t readLE32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint16_t readLE16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

}  // namespace

bool AudioManager::present() const {
  return BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sEs8388 ||
         BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sEs8311 ||
         BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sDac;
}

bool AudioManager::codecWrite(uint8_t reg, uint8_t value) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  Wire.beginTransmission(cfg.codecAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool AudioManager::codecInit() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return true;  // plain I2S DAC, nothing to configure

  const RegVal* seq;
  size_t seqLen;
  const bool es8311 = cfg.output == BoardConfig::AudioOutput::I2sEs8311;
  // ES8311 fed from the MCLK pin (ws397) takes the vendor bring-up; without an
  // MCLK line (M5 PaperColor) it self-clocks from BCLK.
  const bool es8311Mclk = es8311 && cfg.mclk != BoardConfig::PIN_UNASSIGNED;
  if (es8311Mclk) {
    seq = ES8311_INIT_MCLK;
    seqLen = sizeof(ES8311_INIT_MCLK) / sizeof(ES8311_INIT_MCLK[0]);
  } else if (es8311) {
    seq = ES8311_INIT;
    seqLen = sizeof(ES8311_INIT) / sizeof(ES8311_INIT[0]);
  } else {
    seq = ES8388_INIT;
    seqLen = sizeof(ES8388_INIT) / sizeof(ES8388_INIT[0]);
  }

  // A bus another manager already brought up (PMIC / RTC on the same pins)
  // stays as it is: begin() on an initialized TwoWire keeps its clock.
  Wire.begin(cfg.codecSda, cfg.codecScl, CODEC_I2C_HZ);

  // The OEM firmware retries until the codec ACKs; three attempts is plenty
  // for a codec already powered.
  for (int attempt = 0; attempt < 3; ++attempt) {
    bool ok = true;
    if (es8311Mclk) {
      // Vendor reset: hold in reset, release, then the power-on command.
      for (size_t i = 0; i < sizeof(ES8311_RESET_SEQ) / sizeof(ES8311_RESET_SEQ[0]) && ok; ++i) {
        ok = codecWrite(ES8311_RESET_SEQ[i].reg, ES8311_RESET_SEQ[i].val);
        if (i == 0) delay(20);
      }
      if (!ok) {
        delay(100);
        continue;
      }
    }
    for (size_t i = 0; i < seqLen; ++i) {
      if (!codecWrite(seq[i].reg, seq[i].val)) {
        ok = false;
        break;
      }
    }
    if (ok) return true;
    delay(100);
  }
  return false;
}

bool AudioManager::begin() {
  if (begun_) return true;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!present()) return false;

  if (cfg.ampEnable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.ampEnable, OUTPUT);
    digitalWrite(cfg.ampEnable, LOW);  // amp comes up only during playback
  }
  if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.enable, OUTPUT);
    digitalWrite(cfg.enable, cfg.enableActiveHigh ? HIGH : LOW);
    delay(10);  // codec rail ramp before the first I2C access
  }

  if (!codecInit()) {
    log_e("audio codec init failed");
    return false;
  }
  begun_ = true;
  return true;
}

void AudioManager::setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311) {
    // Register 0x32 is logarithmic: dB = -95.5 + 0.5 * N. Scaling the percentage
    // linearly onto N put 70 % at about -23 dB and 85 % at -7.5 dB, which is why
    // the speaker sounded so quiet. Map the percentage onto decibels instead:
    // 1 % = -40 dB, 100 % = +8 dB (0xCF, the vendor maximum), and 70 % lands
    // exactly on 0xB2, the vendor's own default for this 1 W speaker. 0 = mute.
    const uint8_t reg = percent == 0 ? 0 : (uint8_t)(111 + ((uint16_t)percent * 96) / 100);
    codecWrite(0x32, reg);
    return;
  }
  const uint8_t reg = (uint8_t)((uint16_t)percent * ES8388_VOL_MAX_REG / 100);
  // OUT1 (0x2e/0x2f) and OUT2 (0x30/0x31) pairs, like the OEM volume path.
  codecWrite(0x2e, reg);
  codecWrite(0x2f, reg);
  codecWrite(0x30, reg);
  codecWrite(0x31, reg);
}

// DAC mute between alarms so nothing residual reaches the output.
void AudioManager::codecMute(bool mute) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311) {
    codecWrite(0x31, mute ? 0x60 : 0x00);  // DAC_MUTE bits
  } else {
    codecWrite(0x19, mute ? 0x04 : 0x00);  // ES8388 DACCONTROL3 soft mute
  }
}

void AudioManager::setAmp(bool on) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.ampEnable == BoardConfig::PIN_UNASSIGNED) return;
  digitalWrite(cfg.ampEnable, on ? HIGH : LOW);
}

bool AudioManager::captureAvailable() const { return BoardConfig::hasCodecMic(); }

// ES8311 ADC path, per the vendor es8311_microphone_config(): analog MIC1 into
// the PGA at max gain, ADC digital volume 0xC8, modulator + PGA powered. Off
// drops the modulator/PGA power again so an idle mic doesn't burn current.
void AudioManager::codecCapture(bool on) {
  if (!captureAvailable()) return;
  if (on) {
    codecWrite(0x0E, 0x02);  // SYSTEM: analog PGA + ADC modulator on
    codecWrite(0x14, 0x1A);  // SYSTEM: analog mic (LINSEL 1), PGA gain max (30 dB)
    applyMicGain();          // 0x16 ADC_SCALE + 0x17 ADC_VOLUME, según setMicGain()
  } else {
    codecWrite(0x0E, 0x00);  // SYSTEM: ADC modulator + PGA off
  }
}

// ws397: el micrófono se oía flojísimo (8 % de pico hablándole a 15 cm) y el
// dictado llegaba al servidor apenas audible. El PGA analógico (reg 0x14, bits
// 3:0) ya estaba en su tope de 30 dB, así que lo único que queda por subir es
// la ganancia digital del ADC, y son dos registros:
//   0x16 ADC_SCALE : 0-7, pasos de 6 dB (de fábrica 4 = +24 dB)
//   0x17 ADC_VOLUME: 0,5 dB por paso (el vendor deja 0xC8 = +4,5 dB)
// El porcentaje se reparte entre los dos, grueso primero y el resto en el fino:
// 0 % deja exactamente lo del vendor y 100 % suma 36 dB (18 dB llevando
// ADC_SCALE de 4 a 7 y otros 18 dB de ADC_VOLUME). Es ganancia digital, no
// mejora la relación señal/ruido, pero el ADC es de 24 bits y de ahí salen
// muestras de 16, así que hasta acá no se pierde nada útil.
void AudioManager::applyMicGain() {
  if (!BoardConfig::hasCodecMic()) return;
  constexpr uint8_t SCALE_BASE = 0x04;  // ADC_SCALE de fábrica: +24 dB
  constexpr uint8_t SCALE_MAX = 0x07;   // tope del campo: +42 dB
  constexpr uint8_t VOL_BASE = 0xC8;    // ADC_VOLUME del vendor: +4,5 dB
  const int halfDb = static_cast<int>(s_micGain) * MIC_GAIN_MAX_DB * 2 / 100;  // en medios dB
  int coarse = halfDb / 12;  // cada paso de ADC_SCALE son 6 dB = 12 medios dB
  if (coarse > SCALE_MAX - SCALE_BASE) coarse = SCALE_MAX - SCALE_BASE;
  int fine = halfDb - coarse * 12;
  if (fine > 0xFF - VOL_BASE) fine = 0xFF - VOL_BASE;
  codecWrite(0x16, static_cast<uint8_t>(SCALE_BASE + coarse));
  codecWrite(0x17, static_cast<uint8_t>(VOL_BASE + fine));
}

void AudioManager::setMicGain(uint8_t percent) {
  if (percent > 100) percent = 100;
  s_micGain = percent;
  // En caliente: si el micrófono está abierto, el cambio se oye en la próxima
  // muestra. Con el códec dormido el I2C no contesta y no pasa nada: el valor
  // queda guardado y lo aplica el siguiente beginCapture().
  applyMicGain();
}

AudioManager::~AudioManager() {
  if (s_portOwner == this) end();
}

void AudioManager::silenceAmp() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.ampEnable == BoardConfig::PIN_UNASSIGNED) return;
  pinMode(cfg.ampEnable, OUTPUT);
  digitalWrite(cfg.ampEnable, LOW);
}

void AudioManager::powerDown() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!begun_) return;
  stop();
  // El riel del códec también es compartido: cortarlo mientras otra instancia
  // tiene el puerto deja sonando a nadie. Se marca como apagada esta instancia
  // y se deja el hardware como está.
  if (otherOwnsPort()) {
    begun_ = false;
    return;
  }
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311) {
    // Like M5Unified's disable path: drop the amp and the codec rail.
    setAmp(false);
    if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
      digitalWrite(cfg.enable, cfg.enableActiveHigh ? LOW : HIGH);
    }
  } else if (cfg.codecAddr != 0) {
    codecWrite(0x02, 0xff);  // ES8388 CHIPPOWER: everything off
  }
  begun_ = false;
}

bool AudioManager::parseWavHeader(const WavSource& source, WavInfo& info) {
  uint8_t hdr[12];
  if (!source.seek(0)) return false;
  if (source.read(hdr, 12) != 12) return false;
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  size_t pos = 12;
  bool haveFmt = false;
  for (int guard = 0; guard < 32; ++guard) {
    uint8_t chunk[8];
    if (!source.seek(pos) || source.read(chunk, 8) != 8) return false;
    const uint32_t size = readLE32(chunk + 4);
    pos += 8;

    if (memcmp(chunk, "fmt ", 4) == 0) {
      uint8_t fmt[16];
      if (size < 16 || source.read(fmt, 16) != 16) return false;
      const uint16_t audioFormat = readLE16(fmt);
      info.channels = readLE16(fmt + 2);
      info.sampleRate = readLE32(fmt + 4);
      info.bitsPerSample = readLE16(fmt + 14);
      if (audioFormat != 1) return false;  // PCM only
      haveFmt = true;
    } else if (memcmp(chunk, "data", 4) == 0) {
      info.dataStart = pos;
      info.dataLength = size;
      break;
    }
    pos += size + (size & 1);  // chunks are word-aligned
  }

  return haveFmt && info.dataStart != 0 && info.bitsPerSample == 16 &&
         (info.channels == 1 || info.channels == 2) && info.sampleRate >= 8000 &&
         info.sampleRate <= 48000;
}

bool AudioManager::ensureI2s(uint32_t sampleRate) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  i2s_chan_handle_t rx = (i2s_chan_handle_t)rxChan_;

  if (tx && currentRate_ == sampleRate) {
    // Channel exists but was disabled when the last playback drained.
    if (!chanEnabled_) {
      if (i2s_channel_enable(tx) != ESP_OK) return false;
      chanEnabled_ = true;
    }
    return true;
  }

  if (tx) {
    // Rate change on the shared port: both channels must be disabled while the
    // clock is reconfigured (RX borrows TX's BCLK/WS in full duplex).
    if (chanEnabled_) i2s_channel_disable(tx);
    chanEnabled_ = false;
    if (rx && rxEnabled_) i2s_channel_disable(rx);
    rxEnabled_ = false;
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
    clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // codec runs 256x MCLK/LRCK
    if (i2s_channel_reconfig_std_clock(tx, &clk) != ESP_OK) return false;
    if (rx && i2s_channel_reconfig_std_clock(rx, &clk) != ESP_OK) return false;
    if (i2s_channel_enable(tx) != ESP_OK) return false;
    chanEnabled_ = true;
    if (rx && capturing_) {
      if (i2s_channel_enable(rx) != ESP_OK) return false;
      rxEnabled_ = true;
    }
    currentRate_ = sampleRate;
    return true;
  }

  // Nadie más puede tener el puerto: si otra instancia lo tiene, se le pide.
  // end() la deja limpia (para su reproducción, suelta la captura, apaga el
  // códec y borra sus canales), que es exactamente lo que hace falta para que
  // i2s_new_channel de abajo tenga con qué. Sin esto, la primera instancia que
  // sonó en toda la sesión se quedaba con I2S_NUM_0 y ninguna otra volvía a
  // conseguirlo: stop() no suelta los canales, sólo end().
  if (s_portOwner && s_portOwner != this) {
    log_w("i2s: el puerto lo tenía otra instancia, se le pide");
    s_portOwner->end();
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // Without auto_clear the DMA replays its last buffers on underrun — heard
  // as a looping stutter after playback stops.
  chanCfg.auto_clear = true;
  // A codec mic gets its RX channel created together with TX: the driver only
  // pairs the two directions on one port (full duplex) at creation.
  const bool withRx = captureAvailable();
  if (i2s_new_channel(&chanCfg, &tx, withRx ? &rx : nullptr) != ESP_OK) return false;

  i2s_std_config_t std = {};
  std.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
  std.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  std.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  std.gpio_cfg.mclk = cfg.mclk == BoardConfig::PIN_UNASSIGNED ? I2S_GPIO_UNUSED
                                                              : (gpio_num_t)cfg.mclk;
  std.gpio_cfg.bclk = (gpio_num_t)cfg.bclk;
  std.gpio_cfg.ws = (gpio_num_t)cfg.lrclk;
  std.gpio_cfg.dout = (gpio_num_t)cfg.dout;
  // Each channel only claims the data pin of its own direction.
  std.gpio_cfg.din = withRx ? (gpio_num_t)cfg.din : I2S_GPIO_UNUSED;

  auto fail = [&]() {
    i2s_del_channel(tx);
    if (rx) i2s_del_channel(rx);
    return false;
  };
  if (i2s_channel_init_std_mode(tx, &std) != ESP_OK) return fail();
  if (rx && i2s_channel_init_std_mode(rx, &std) != ESP_OK) return fail();
  if (i2s_channel_enable(tx) != ESP_OK) return fail();
  txChan_ = tx;
  rxChan_ = rx;  // stays disabled until beginCapture()
  chanEnabled_ = true;
  rxEnabled_ = false;
  currentRate_ = sampleRate;
  s_portOwner = this;  // desde acá, el puerto es de esta instancia
  return true;
}

void AudioManager::teardownI2s() {
  // Se suelta la propiedad del puerto ANTES de borrar los canales: si esta
  // instancia lo tenía, al salir de acá no lo tiene nadie y la próxima que lo
  // pida lo va a poder crear.
  if (s_portOwner == this) s_portOwner = nullptr;
  if (rxChan_) {
    i2s_chan_handle_t rx = (i2s_chan_handle_t)rxChan_;
    if (rxEnabled_) i2s_channel_disable(rx);
    i2s_del_channel(rx);
    rxChan_ = nullptr;
    rxEnabled_ = false;
  }
  if (!txChan_) return;
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  if (chanEnabled_) i2s_channel_disable(tx);
  i2s_del_channel(tx);
  txChan_ = nullptr;
  chanEnabled_ = false;
  currentRate_ = 0;
}

// Every early return says WHY. A caller can only report "capture failed", and
// from the device — no cable, no serial monitor — that one sentence covered a
// missing mic, a codec that never came up, an I2S port already owned by another
// AudioManager instance, and a channel that refused to enable. Those are four
// different problems with four different fixes, and telling them apart by
// guessing costs a whole test round each time.
bool AudioManager::beginCapture(uint32_t sampleRate) {
  if (!captureAvailable()) {
    s_lastCaptureError = "la placa no tiene micrófono por códec";
    log_e("beginCapture: this board has no codec mic");
    return false;
  }
  if (sampleRate < 8000 || sampleRate > 48000) {
    s_lastCaptureError = "tasa de muestreo fuera de rango";
    log_e("beginCapture: rate %u out of range", (unsigned)sampleRate);
    return false;
  }
  if (!begun_ && !begin()) {
    s_lastCaptureError = "el códec no levantó (begin)";
    log_e("beginCapture: begin() failed (codec did not come up)");
    return false;
  }
  if (capturing_ && currentRate_ == sampleRate) return true;
  // One clock per port: playback (if any) yields, and a running capture at
  // another rate is restarted.
  stop();
  if (capturing_) endCapture();
  if (!ensureI2s(sampleRate) || !rxChan_) {
    s_lastCaptureError = "no se pudo armar el I2S (puerto ocupado o sin canal RX)";
    log_e("beginCapture: i2s setup failed at %u Hz (port busy or no RX channel)", (unsigned)sampleRate);
    return false;
  }
  codecCapture(true);
  if (!rxEnabled_) {
    const esp_err_t err = i2s_channel_enable((i2s_chan_handle_t)rxChan_);
    if (err != ESP_OK) {
      s_lastCaptureError = "el canal RX no se pudo habilitar";
      log_e("beginCapture: i2s_channel_enable(rx) -> %d", (int)err);
      codecCapture(false);
      return false;
    }
    rxEnabled_ = true;
  }
  capturing_ = true;
  s_lastCaptureError = nullptr;
  return true;
}

int AudioManager::readCapture(int16_t* dst, size_t maxSamples, uint32_t timeoutMs) {
  if (!capturing_ || !rxChan_ || !dst || maxSamples == 0) return -1;
  i2s_chan_handle_t rx = (i2s_chan_handle_t)rxChan_;
  // The port runs stereo 16-bit frames (shared with playback); the ADC data
  // rides the left slot. De-interleave in blocks small enough for the stack.
  constexpr size_t FRAMES = 128;
  int16_t frames[FRAMES * 2];
  size_t got = 0;
  while (got < maxSamples) {
    size_t want = maxSamples - got;
    if (want > FRAMES) want = FRAMES;
    size_t bytes = 0;
    const esp_err_t err = i2s_channel_read(rx, frames, want * 4, &bytes, pdMS_TO_TICKS(timeoutMs));
    const size_t n = bytes / 4;
    for (size_t i = 0; i < n; ++i) dst[got + i] = frames[i * 2];
    got += n;
    if (err == ESP_ERR_TIMEOUT) break;
    if (err != ESP_OK) return got ? (int)got : -1;
    if (n < want) break;
  }
  return (int)got;
}

void AudioManager::endCapture() {
  if (!capturing_) return;
  capturing_ = false;
  if (rxChan_ && rxEnabled_) i2s_channel_disable((i2s_chan_handle_t)rxChan_);
  rxEnabled_ = false;
  codecCapture(false);
}

void AudioManager::end() {
  stop();
  endCapture();
  powerDown();
  teardownI2s();
}

bool AudioManager::play(const WavSource& source, bool loop) {
  if (!begun_ && !begin()) return false;
  stop();

  WavInfo info;
  if (!parseWavHeader(source, info)) {
    log_e("unsupported WAV (need 16-bit PCM, 1-2ch, 8-48 kHz)");
    return false;
  }
  if (!ensureI2s(info.sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }
  if (!source.seek(info.dataStart)) return false;

  // Unmute the DAC (stop() mutes it). Codec writes stay on the caller's core
  // so the shared I2C bus is never touched from the audio task. The speaker
  // amp comes up in the playback task once silence is flowing — enabling it
  // against an idle I2S line is an audible pop.
  codecMute(false);

  source_ = source;
  wav_ = info;
  loop_ = loop;
  stopRequested_ = false;
  paused_ = false;
  pausedIdle_ = false;
  playing_ = true;

  // Same shape as the OEM "musicTask" (high priority, core 0 — the Arduino
  // loop owns core 1); 8K stack covers the on-stack sample buffers.
  if (xTaskCreatePinnedToCore(taskEntry, "audio_play", 8192, this, 10, &task_, 0) != pdPASS) {
    playing_ = false;
    task_ = nullptr;
    return false;
  }
  return true;
}

bool AudioManager::playBuffer(const uint8_t* data, size_t len, bool loop) {
  // Shared offset state lives in the lambdas; play() copies them.
  auto offset = std::make_shared<size_t>(0);
  WavSource src;
  src.read = [data, len, offset](uint8_t* dst, size_t want) -> int {
    const size_t left = len - *offset;
    const size_t n = want < left ? want : left;
    memcpy(dst, data + *offset, n);
    *offset += n;
    return (int)n;
  };
  src.seek = [len, offset](size_t pos) {
    if (pos > len) return false;
    *offset = pos;
    return true;
  };
  return play(src, loop);
}

void AudioManager::stop() {
  // Levantar la pausa primero: una tarea pausada duerme de a 100 ms sin tocar
  // el I2S, y el bucle de espera de abajo la dejaría llegar al tope.
  paused_ = false;
  if (playing_) {
    stopRequested_ = true;
    // The task deletes itself; wait for it to drain (bounded).
    for (int i = 0; i < 200 && playing_; ++i) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  // Drop the amp and mute the DAC so nothing residual reaches the output
  // between alarms. A stop() after a capture-only session (the amp is enabled
  // by begin(), not by playback) used to leave the amplifier powered and
  // hissing until the next play.
  //
  // PERO no si el puerto lo tiene OTRA instancia: el amplificador y el códec
  // son UNO SOLO para toda la placa. Cada clase tiene su AudioManager (clics,
  // pitidos, voz, música, micrófono) y la que perdió el puerto seguía bajando
  // el amp y muteando el DAC por debajo de la que está sonando — la música se
  // quedaba muda sin que nadie la hubiera parado.
  if (!otherOwnsPort()) {
    setAmp(false);
    codecMute(true);
  }
}

// El puerto lo tiene otra instancia viva: lo que sea del códec o del
// amplificador no es nuestro para tocarlo.
bool AudioManager::otherOwnsPort() const { return s_portOwner != nullptr && s_portOwner != this; }

bool AudioManager::portBusy() { return s_portOwner != nullptr && (s_portOwner->playing_ || s_portOwner->capturing_); }

void AudioManager::setPaused(const bool paused) {
  if (paused_ == paused) return;
  paused_ = paused;
  // El trabajo lo hace la tarea (es la dueña del canal TX): acá sólo se
  // levanta la bandera y, al pausar, se espera un momento a que la tarea deje
  // la línea en silencio para que el amplificador no se apague con señal
  // encima (chasquido del clase D).
  if (paused) {
    for (int i = 0; i < 50 && playing_ && !pausedIdle_; ++i) vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void AudioManager::taskEntry(void* self) { static_cast<AudioManager*>(self)->taskLoop(); }

void AudioManager::taskLoop() {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  uint8_t inBuf[READ_CHUNK];
  // Mono is duplicated into both slots, so the out buffer is 2x.
  int16_t outBuf[READ_CHUNK];

  // Prime the line with silence, then raise the amp: the AW8737A pops loudly
  // when enabled against an idle or just-started I2S line.
  //
  // El silencio se mide en MILISEGUNDOS, no en buffers. Cada escritura son 512
  // cuadros: a 16 kHz eso son 32 ms, pero a 48 kHz son 10,7 ms, y los clics de
  // la interfaz se generan justo a 48 kHz. Con dos buffers el amplificador
  // recien estaba arrancando cuando ya habia terminado el clic de 26 ms: se oia
  // apenas o no se oia nada. Con los milisegundos fijos da igual la tasa.
  const uint32_t rate = currentRate_ > 0 ? currentRate_ : 16000;
  const size_t framesPerWrite = sizeof(outBuf) / (2 * sizeof(int16_t));
  const auto writesFor = [&](const uint32_t ms) {
    const size_t frames = static_cast<size_t>(rate) * ms / 1000;
    return static_cast<int>((frames + framesPerWrite - 1) / framesPerWrite);
  };
  memset(outBuf, 0, sizeof(outBuf));
  const int preWrites = writesFor(AMP_PRIME_MS);
  for (int i = 0; i < preWrites; ++i) {
    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
  }
  setAmp(true);
  // Y otro tanto DESPUES de levantar el amplificador. Un clase D no pasa de
  // apagado a amplificando en cero: tarda unas decenas de ms en arrancar, y en
  // ese rato los primeros samples reales salen mudos. Con clips cortos —el
  // nombre de una carta, un aviso de dos palabras— eso se oye como que empieza
  // tarde o que se come la primera sílaba. El silencio es gratis y va antes.
  const int postWrites = writesFor(AMP_SETTLE_MS);
  for (int i = 0; i < postWrites; ++i) {
    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
  }

  size_t consumed = 0;
  bool wasPaused = false;
  while (!stopRequested_) {
    if (paused_) {
      if (!wasPaused) {
        // Vaciar los descriptores del DMA con silencio ANTES de bajar el amp y
        // apagar el canal: un canal meramente detenido repite lo último que
        // quedó en el DMA, y el clase D chasquea si se apaga con señal.
        memset(outBuf, 0, sizeof(outBuf));
        for (int i = 0; i < 4; ++i) {
          size_t written = 0;
          if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
        }
        setAmp(false);
        if (chanEnabled_ && !capturing_) {
          i2s_channel_disable(tx);
          chanEnabled_ = false;
        }
        wasPaused = true;
        pausedIdle_ = true;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (wasPaused) {
      pausedIdle_ = false;
      if (!chanEnabled_) {
        if (i2s_channel_enable(tx) != ESP_OK) break;
        chanEnabled_ = true;
      }
      memset(outBuf, 0, sizeof(outBuf));
      for (int i = 0; i < 2; ++i) {
        size_t written = 0;
        if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
      }
      setAmp(true);
      wasPaused = false;
    }
    size_t want = READ_CHUNK;
    if (wav_.dataLength > 0) {
      const size_t left = wav_.dataLength - consumed;
      if (left == 0) {
        if (loop_ && source_.seek(wav_.dataStart)) {
          consumed = 0;
          continue;
        }
        break;
      }
      if (want > left) want = left;
    }
    want &= ~(size_t)3;  // keep sample alignment (16-bit stereo frames)
    if (want < 4) want = 4;

    const int n = source_.read(inBuf, want);
    if (n <= 0) {
      if (loop_ && source_.seek(wav_.dataStart)) {
        consumed = 0;
        continue;
      }
      break;
    }
    consumed += n;

    const int16_t* samples = (const int16_t*)inBuf;
    size_t outBytes;
    if (wav_.channels == 1) {
      const int frames = n / 2;
      for (int i = 0; i < frames; ++i) {
        outBuf[i * 2] = samples[i];
        outBuf[i * 2 + 1] = samples[i];
      }
      outBytes = (size_t)frames * 4;
    } else {
      memcpy(outBuf, inBuf, n);
      outBytes = (size_t)n;
    }

    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, outBytes, &written, pdMS_TO_TICKS(1000)) != ESP_OK) break;
  }

  // Flush silence through every DMA descriptor, then stop the channel
  // entirely — a merely-idle channel replays stale DMA contents (stutter).
  // Except while a capture runs: RX takes its BCLK/WS from TX, so TX stays
  // enabled (auto_clear keeps it on silence).
  memset(outBuf, 0, sizeof(outBuf));
  for (int i = 0; i < 6; ++i) {
    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
  }
  // Drop the amplifier here, not only in stop(), and BEFORE the channel goes
  // down. A track that ended on its own used to leave the class-D enable HIGH
  // while the I2S channel was disabled right under it, so the codec sat with no
  // MCLK/BCLK and an undefined analog output and the amp faithfully amplified
  // that — a permanent hiss lasting until something called stop() or
  // powerDown(). The order matters: silence has just been flushed through the
  // DMA, so cutting the enable now lands in a quiet moment; cutting it after
  // i2s_channel_disable() would leave a window of clockless codec into a live
  // amp, which is the burst of noise this is meant to avoid. Unconditional:
  // when a capture keeps TX enabled for its BCLK, the speaker still has no
  // business being live.
  setAmp(false);

  if (!capturing_ && chanEnabled_) {
    i2s_channel_disable(tx);
    chanEnabled_ = false;
  }

  paused_ = false;
  pausedIdle_ = false;
  playing_ = false;
  task_ = nullptr;
  vTaskDelete(nullptr);
}

}  // namespace freeink

#else  // !FREEINK_CAP_AUDIO — stubs so callers need no #ifdefs

namespace freeink {
bool AudioManager::present() const { return false; }
bool AudioManager::begin() { return false; }
void AudioManager::setVolume(uint8_t) {}
bool AudioManager::play(const WavSource&, bool) { return false; }
bool AudioManager::playBuffer(const uint8_t*, size_t, bool) { return false; }
void AudioManager::stop() {}
void AudioManager::setPaused(bool) {}
AudioManager::~AudioManager() {}
void AudioManager::powerDown() {}
void AudioManager::silenceAmp() {}
void AudioManager::end() {}
bool AudioManager::captureAvailable() const { return false; }
bool AudioManager::beginCapture(uint32_t) { return false; }
int AudioManager::readCapture(int16_t*, size_t, uint32_t) { return -1; }
void AudioManager::endCapture() {}
void AudioManager::codecCapture(bool) {}
void AudioManager::applyMicGain() {}
void AudioManager::setMicGain(uint8_t percent) { s_micGain = percent > 100 ? 100 : percent; }
bool AudioManager::parseWavHeader(const WavSource&, WavInfo&) { return false; }
bool AudioManager::ensureI2s(uint32_t) { return false; }
void AudioManager::teardownI2s() {}
bool AudioManager::codecInit() { return false; }
bool AudioManager::codecWrite(uint8_t, uint8_t) { return false; }
void AudioManager::codecMute(bool) {}
void AudioManager::setAmp(bool) {}
void AudioManager::taskEntry(void*) {}
void AudioManager::taskLoop() {}
}  // namespace freeink

#endif  // FREEINK_CAP_AUDIO
