/*
 * Knob Sound Device
 * -----------------
 * ESP32 + AS5600 (magnetic angle sensor) + MAX98357A (I2S amp) + WS2812 strip
 *
 * - The knob rotates a full 360 degrees. The circle is split into NUM_ZONES
 *   equal zones. Entering a zone plays that zone's sound (/z0.wav, /z1.wav ...).
 * - The same angle ALWAYS gives the same sound (predictable).
 * - Sounds fade in/out (no clicks or sudden cuts), volume is hard-capped.
 * - LED strip: dim glow in the zone colour, a pointer showing the knob
 *   position, a soft flash when a zone is entered, gentle breathing while
 *   a sound plays, amber blink if the sensor/magnet has a problem.
 *
 * REQUIREMENTS
 *   - Arduino-ESP32 core 3.x (uses the ESP_I2S library that ships with it)
 *   - Library: "Adafruit NeoPixel" (Library Manager)
 *   - Partition scheme with a data partition (e.g. "Default 4MB with spiffs")
 *   - Audio files uploaded to LittleFS (see README notes in the chat reply)
 *
 * AUDIO FILE FORMAT (all files must match)
 *   WAV, PCM, 16-bit, mono or stereo, sample rate = SAMPLE_RATE below.
 *   Convert with:
 *     ffmpeg -i input.mp3 -ar 22050 -ac 1 -c:a pcm_s16le z0.wav
 *
 * WIRING (ESP32 DevKit, change the pins below if you wish)
 *   AS5600   : VCC 3.3V, GND, SDA -> GPIO21, SCL -> GPIO22
 *   MAX98357A: VIN 5V (or 3.3V), GND, BCLK -> GPIO26, LRC -> GPIO25, DIN -> GPIO27
 *              SD pin: leave as the breakout default (mono mix L+R), or set
 *              PIN_AMP_SD to a GPIO to control shutdown from firmware.
 *   WS2812   : DIN -> GPIO4 (through ~330 ohm), 5V from a proper supply,
 *              COMMON GROUND with the ESP32. Add a 1000 uF cap across 5V/GND.
 */

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ESP_I2S.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

// ============================================================================
//  CONFIGURATION
// ============================================================================

// ---- Pins ------------------------------------------------------------------
constexpr int PIN_I2C_SDA   = 21;
constexpr int PIN_I2C_SCL   = 22;
constexpr int PIN_I2S_BCLK  = 26;
constexpr int PIN_I2S_LRC   = 25;
constexpr int PIN_I2S_DOUT  = 27;
constexpr int PIN_AMP_SD    = -1;   // -1 = not used. Otherwise GPIO driving MAX98357A SD (HIGH = on)
constexpr int PIN_LED       = 4;

// ---- Knob zones ------------------------------------------------------------
constexpr int   NUM_ZONES       = 8;      // equal zones around 360 degrees
constexpr float ZONE_OFFSET_DEG = 0.0f;   // rotate where zone 0 starts (degrees)
constexpr bool  INVERT_DIRECTION = false; // set true if increasing angle is the wrong way
constexpr float HYSTERESIS_DEG  = 3.0f;   // dead band at zone edges
constexpr uint32_t DWELL_MS     = 60;     // angle must stay in new zone this long
constexpr bool  PLAY_ON_BOOT    = false;  // play the sound of the starting zone at power-up?

// ---- Sensor ----------------------------------------------------------------
constexpr uint32_t SENSE_INTERVAL_MS  = 10;   // angle read rate (100 Hz)
constexpr uint32_t STATUS_INTERVAL_MS = 250;  // magnet check rate
constexpr float    FILTER_ALPHA       = 0.35f; // 0..1, lower = smoother
constexpr int      I2C_FAIL_LIMIT     = 5;     // consecutive failures => sensor error

// ---- Audio -----------------------------------------------------------------
constexpr uint32_t SAMPLE_RATE   = 22050;  // all WAV files must use this rate
constexpr float    MASTER_VOLUME = 0.40f;  // 0.0 .. 1.0  (also the hard cap)
constexpr uint32_t FADE_MS       = 60;     // fade in / fade out time
constexpr int      CHUNK_FRAMES  = 256;    // frames processed per I2S write

// ---- LEDs ------------------------------------------------------------------
constexpr int     LED_COUNT          = 16;
constexpr uint8_t LED_MAX_BRIGHTNESS = 60;   // 0..255 global cap (protects eyes and power supply)
constexpr uint32_t LED_FRAME_MS      = 20;   // 50 fps
constexpr uint32_t FLASH_MS          = 300;  // zone-enter flash length

struct RGB { uint8_t r, g, b; };

// One calm colour per zone (must have exactly NUM_ZONES entries)
constexpr RGB ZONE_COLORS[] = {
  {255,  60,  60},   // 0 soft red
  {255, 140,  30},   // 1 orange
  {255, 220,  40},   // 2 yellow
  { 60, 220,  80},   // 3 green
  { 40, 220, 200},   // 4 teal
  { 60, 120, 255},   // 5 blue
  {150,  80, 255},   // 6 violet
  {255,  90, 190},   // 7 pink
};
static_assert(sizeof(ZONE_COLORS) / sizeof(ZONE_COLORS[0]) == NUM_ZONES,
              "ZONE_COLORS must have exactly NUM_ZONES entries");

constexpr RGB COLOR_READY  = {  0, 200,  60};
constexpr RGB COLOR_ERROR  = {255, 120,   0};  // amber: sensor/magnet problem
constexpr RGB COLOR_NOFILE = {200,   0,  40};  // sound file missing/invalid
constexpr RGB COLOR_IDLE   = { 60,  60,  60};

// Set to 1 to print angle/zone to Serial (115200)
#define DEBUG_ANGLE 0

// ============================================================================
//  GLOBALS
// ============================================================================

I2SClass i2s;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

QueueHandle_t audioQueue = nullptr;
volatile bool     audioPlaying    = false;
volatile uint32_t audioErrorCount = 0;

// ============================================================================
//  AS5600
// ============================================================================

constexpr uint8_t AS5600_ADDR       = 0x36;
constexpr uint8_t AS5600_REG_STATUS = 0x0B;
constexpr uint8_t AS5600_REG_RAWANG = 0x0C;

bool as5600ReadRaw(uint16_t &raw) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_RAWANG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = ((uint16_t)(hi & 0x0F) << 8) | lo;
  return true;
}

bool as5600ReadStatus(uint8_t &status) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_STATUS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return false;
  status = Wire.read();
  return true;
}

// Status bits: MD (0x20) magnet detected, ML (0x10) too weak, MH (0x08) too strong
inline bool magnetDetected(uint8_t s) { return (s & 0x20) != 0; }

// ============================================================================
//  ANGLE MATH
// ============================================================================

float wrap360(float x) {
  x = fmodf(x, 360.0f);
  if (x < 0.0f) x += 360.0f;
  return x;
}

// shortest signed difference a - b, in (-180, 180]
float wrapDiff(float a, float b) {
  float d = fmodf(a - b, 360.0f);
  if (d > 180.0f)  d -= 360.0f;
  if (d <= -180.0f) d += 360.0f;
  return d;
}

// Zone for an angle, with hysteresis around the currently held zone.
int desiredZone(float shiftedAngle, int heldZone) {
  const float W = 360.0f / NUM_ZONES;
  if (heldZone >= 0) {
    float center = (heldZone + 0.5f) * W;
    if (fabsf(wrapDiff(shiftedAngle, center)) <= (W * 0.5f + HYSTERESIS_DEG)) {
      return heldZone;  // still inside held zone (plus dead band)
    }
  }
  int z = (int)(shiftedAngle / W);
  if (z < 0) z = 0;
  if (z >= NUM_ZONES) z = NUM_ZONES - 1;
  return z;
}

// ============================================================================
//  AUDIO (runs in its own task on core 0)
// ============================================================================

File     wavFile;
uint32_t wavRemainingBytes = 0;
uint16_t wavChannels       = 1;

static int16_t audioIn [CHUNK_FRAMES * 2];  // up to stereo input
static int16_t audioOut[CHUNK_FRAMES * 2];  // always stereo output

void closeWav() {
  if (wavFile) wavFile.close();
  wavRemainingBytes = 0;
}

// Write all bytes to I2S (blocking, paced by the DMA).
void i2sWriteAll(const uint8_t *data, size_t bytes) {
  size_t offset = 0;
  int fails = 0;
  while (offset < bytes) {
    size_t w = i2s.write(data + offset, bytes - offset);
    if (w == 0) {
      if (++fails > 50) return;  // give up, avoid locking the task
      vTaskDelay(1);
    } else {
      fails = 0;
      offset += w;
    }
  }
}

// ~10 ms of silence so the DMA never replays stale audio
void writeSilence() {
  const int frames = SAMPLE_RATE / 100;
  memset(audioOut, 0, sizeof(audioOut));
  int remaining = frames;
  while (remaining > 0) {
    int n = remaining > CHUNK_FRAMES ? CHUNK_FRAMES : remaining;
    i2sWriteAll((const uint8_t *)audioOut, (size_t)n * 4);
    remaining -= n;
  }
}

// Open /z<zone>.wav, validate the header, position at the PCM data.
bool openWav(int zone) {
  closeWav();

  char path[24];
  snprintf(path, sizeof(path), "/z%d.wav", zone);

  wavFile = LittleFS.open(path, "r");
  if (!wavFile) {
    Serial.printf("[audio] missing file %s\n", path);
    return false;
  }

  uint8_t hdr[12];
  if (wavFile.read(hdr, 12) != 12 || memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
    Serial.printf("[audio] %s is not a RIFF/WAVE file\n", path);
    closeWav();
    return false;
  }

  bool fmtOk = false;
  bool dataFound = false;

  while (wavFile.available()) {
    uint8_t ch[8];
    if (wavFile.read(ch, 8) != 8) break;
    uint32_t sz = (uint32_t)ch[4] | ((uint32_t)ch[5] << 8) | ((uint32_t)ch[6] << 16) | ((uint32_t)ch[7] << 24);

    if (memcmp(ch, "fmt ", 4) == 0) {
      if (sz < 16) break;
      uint8_t fm[16];
      if (wavFile.read(fm, 16) != 16) break;
      uint16_t format   = fm[0]  | (fm[1]  << 8);
      uint16_t channels = fm[2]  | (fm[3]  << 8);
      uint32_t rate     = (uint32_t)fm[4] | ((uint32_t)fm[5] << 8) | ((uint32_t)fm[6] << 16) | ((uint32_t)fm[7] << 24);
      uint16_t bits     = fm[14] | (fm[15] << 8);

      if (format != 1 || (channels != 1 && channels != 2) || bits != 16 || rate != SAMPLE_RATE) {
        Serial.printf("[audio] %s unsupported: fmt=%u ch=%u rate=%lu bits=%u (need PCM, 1-2 ch, %lu Hz, 16-bit)\n",
                      path, format, channels, (unsigned long)rate, bits, (unsigned long)SAMPLE_RATE);
        closeWav();
        return false;
      }
      wavChannels = channels;
      fmtOk = true;
      uint32_t skip = (sz - 16) + (sz & 1);
      if (skip) wavFile.seek(wavFile.position() + skip);
    } else if (memcmp(ch, "data", 4) == 0) {
      if (!fmtOk) break;
      uint32_t avail = wavFile.size() - wavFile.position();
      wavRemainingBytes = (sz > avail) ? avail : sz;
      dataFound = true;
      break;
    } else {
      wavFile.seek(wavFile.position() + sz + (sz & 1));  // skip LIST, etc.
    }
  }

  if (!fmtOk || !dataFound || wavRemainingBytes < (uint32_t)(wavChannels * 2)) {
    Serial.printf("[audio] %s has no usable data\n", path);
    closeWav();
    return false;
  }
  return true;
}

enum PlayState { PS_IDLE, PS_PLAYING, PS_FADING_OUT };

void audioTask(void *) {
  PlayState st = PS_IDLE;
  int   pending = -1;
  float gain = 0.0f;
  float target = 1.0f;

  const uint32_t fadeFrames = (SAMPLE_RATE * FADE_MS) / 1000;
  const float    step       = 1.0f / (float)(fadeFrames > 0 ? fadeFrames : 1);

  for (;;) {
    // ---- new request? ----
    int8_t cmd;
    TickType_t waitTicks = (st == PS_IDLE) ? pdMS_TO_TICKS(20) : 0;
    if (xQueueReceive(audioQueue, &cmd, waitTicks) == pdTRUE) {
      if (st == PS_IDLE) {
        if (openWav(cmd)) {
          st = PS_PLAYING; gain = 0.0f; target = 1.0f; audioPlaying = true;
        } else {
          audioErrorCount = audioErrorCount + 1;
        }
      } else {
        pending = cmd;       // latest request wins
        st = PS_FADING_OUT;  // fade the current sound out first
        target = 0.0f;
      }
    }

    if (st == PS_IDLE) continue;

    // ---- read one chunk ----
    const size_t frameBytes = (size_t)wavChannels * 2;
    uint32_t wantBytes = (uint32_t)CHUNK_FRAMES * frameBytes;
    if (wantBytes > wavRemainingBytes) wantBytes = wavRemainingBytes;

    size_t got = wavFile.read((uint8_t *)audioIn, wantBytes);
    got -= got % frameBytes;
    const uint32_t frames = (uint32_t)(got / frameBytes);

    if (frames > 0) {
      const uint32_t framesLeftAtStart = wavRemainingBytes / frameBytes;

      for (uint32_t i = 0; i < frames; i++) {
        int32_t s;
        if (wavChannels == 1) {
          s = audioIn[i];
        } else {
          s = ((int32_t)audioIn[2 * i] + (int32_t)audioIn[2 * i + 1]) / 2;
        }

        // start fading out shortly before the end of the file
        if (st == PS_PLAYING && (framesLeftAtStart - i) <= fadeFrames) target = 0.0f;

        if (gain < target)      { gain += step; if (gain > target) gain = target; }
        else if (gain > target) { gain -= step; if (gain < target) gain = target; }

        float v = (float)s * gain * MASTER_VOLUME;
        if (v > 32767.0f)  v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        int16_t o = (int16_t)v;
        audioOut[2 * i]     = o;
        audioOut[2 * i + 1] = o;
      }

      wavRemainingBytes -= (uint32_t)got;
      i2sWriteAll((const uint8_t *)audioOut, (size_t)frames * 4);
    }

    // ---- finished? ----
    bool finished = (frames == 0) || (wavRemainingBytes == 0) ||
                    (st == PS_FADING_OUT && gain <= 0.0f);

    if (finished) {
      closeWav();
      writeSilence();
      audioPlaying = false;
      st = PS_IDLE;

      if (pending >= 0) {
        int z = pending;
        pending = -1;
        if (openWav(z)) {
          st = PS_PLAYING; gain = 0.0f; target = 1.0f; audioPlaying = true;
        } else {
          audioErrorCount = audioErrorCount + 1;
        }
      }
    }
  }
}

void audioRequest(int zone) {
  int8_t z = (int8_t)zone;
  xQueueOverwrite(audioQueue, &z);  // queue length 1: newest request replaces older one
}

// ============================================================================
//  LEDs
// ============================================================================

RGB      flashColor = COLOR_READY;
uint32_t flashStart = 0;
bool     flashActive = false;

void ledFlash(const RGB &c) {
  flashColor = c;
  flashStart = millis();
  flashActive = true;
}

static inline uint8_t scale8f(uint8_t v, float k) {
  float x = (float)v * k;
  if (x < 0.0f) x = 0.0f;
  if (x > 255.0f) x = 255.0f;
  return (uint8_t)x;
}

void ledRender(uint32_t now, int zone, float angleDeg, bool sensorError) {
  if (sensorError) {
    // gentle amber blink, 1 Hz
    bool on = ((now / 500) % 2) == 0;
    uint32_t col = on ? strip.Color(scale8f(COLOR_ERROR.r, 0.5f),
                                    scale8f(COLOR_ERROR.g, 0.5f),
                                    scale8f(COLOR_ERROR.b, 0.5f))
                      : 0;
    for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, col);
    strip.show();
    return;
  }

  RGB c = (zone >= 0) ? ZONE_COLORS[zone] : COLOR_IDLE;

  // base glow: steady when silent, slow breathing while sound plays
  float k = 0.12f;
  if (audioPlaying) k = 0.20f + 0.10f * sinf((float)now * 0.004f);

  uint8_t fr[LED_COUNT], fg[LED_COUNT], fb[LED_COUNT];
  for (int i = 0; i < LED_COUNT; i++) {
    fr[i] = scale8f(c.r, k);
    fg[i] = scale8f(c.g, k);
    fb[i] = scale8f(c.b, k);
  }

  // pointer showing the physical knob position
  int p = (int)(angleDeg / 360.0f * LED_COUNT);
  if (p < 0) p = 0;
  if (p >= LED_COUNT) p = LED_COUNT - 1;
  fr[p] = c.r; fg[p] = c.g; fb[p] = c.b;

  // event flash (zone entered, ready, file error)
  if (flashActive) {
    uint32_t age = now - flashStart;
    if (age >= FLASH_MS) {
      flashActive = false;
    } else {
      float f = 1.0f - (float)age / (float)FLASH_MS;
      f *= 0.9f;
      uint8_t r = scale8f(flashColor.r, f);
      uint8_t g = scale8f(flashColor.g, f);
      uint8_t b = scale8f(flashColor.b, f);
      for (int i = 0; i < LED_COUNT; i++) {
        if (r > fr[i]) fr[i] = r;
        if (g > fg[i]) fg[i] = g;
        if (b > fb[i]) fb[i] = b;
      }
    }
  }

  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, fr[i], fg[i], fb[i]);
  strip.show();
}

// ============================================================================
//  STATE
// ============================================================================

float    filteredAngle = 0.0f;
bool     angleValid    = false;
bool     sensorError   = false;
int      i2cFailures   = 0;
int      currentZone   = -1;
int      pendingZone   = -1;
uint32_t pendingSince  = 0;
uint32_t lastSense     = 0;
uint32_t lastStatus    = 0;
uint32_t lastLed       = 0;
uint32_t seenAudioErrors = 0;

void enterSensorError() {
  if (!sensorError) Serial.println("[sensor] error: AS5600 not responding or magnet missing");
  sensorError  = true;
  angleValid   = false;
  currentZone  = -1;
  pendingZone  = -1;
}

void leaveSensorError() {
  if (sensorError) Serial.println("[sensor] OK");
  sensorError = false;
  i2cFailures = 0;
  ledFlash(COLOR_READY);
}

void triggerZone(int zone) {
  Serial.printf("[zone] -> %d\n", zone);
  audioRequest(zone);
  ledFlash(ZONE_COLORS[zone]);
}

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nKnob Sound Device starting");

  // LEDs first so we can show status
  strip.begin();
  strip.setBrightness(LED_MAX_BRIGHTNESS);
  strip.clear();
  strip.show();

  // I2C / AS5600
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  // Filesystem
  if (!LittleFS.begin(false)) {
    Serial.println("[fs] LittleFS mount failed (did you upload the sound files / pick a partition scheme with data?)");
    ledFlash(COLOR_NOFILE);
  } else {
    for (int z = 0; z < NUM_ZONES; z++) {
      char path[24];
      snprintf(path, sizeof(path), "/z%d.wav", z);
      if (!LittleFS.exists(path)) Serial.printf("[fs] WARNING: %s not found\n", path);
    }
  }

  // Amp shutdown pin (optional)
  if (PIN_AMP_SD >= 0) {
    pinMode(PIN_AMP_SD, OUTPUT);
    digitalWrite(PIN_AMP_SD, LOW);
  }

  // I2S (stereo 16-bit, same sample on both slots so the MAX98357A gets full level)
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_LRC, PIN_I2S_DOUT);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("[i2s] failed to start I2S");
    while (true) {  // fatal: show error pattern forever
      ledRender(millis(), -1, 0.0f, true);
      delay(LED_FRAME_MS);
    }
  }
  if (PIN_AMP_SD >= 0) digitalWrite(PIN_AMP_SD, HIGH);

  // Audio task
  audioQueue = xQueueCreate(1, sizeof(int8_t));
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 2, nullptr, 0);

  // Initial sensor check
  uint8_t st = 0;
  if (!as5600ReadStatus(st) || !magnetDetected(st)) {
    enterSensorError();
  } else {
    ledFlash(COLOR_READY);
  }

  Serial.println("Ready");
}

// ============================================================================
//  LOOP
// ============================================================================

void loop() {
  uint32_t now = millis();

  // ---- magnet / presence check ----
  if (now - lastStatus >= STATUS_INTERVAL_MS) {
    lastStatus = now;
    uint8_t status;
    if (!as5600ReadStatus(status) || !magnetDetected(status)) {
      enterSensorError();
    } else if (sensorError) {
      leaveSensorError();
    }
  }

  // ---- angle read ----
  if (!sensorError && (now - lastSense >= SENSE_INTERVAL_MS)) {
    lastSense = now;
    uint16_t raw;
    if (as5600ReadRaw(raw)) {
      i2cFailures = 0;
      float a = (float)raw * (360.0f / 4096.0f);
      if (INVERT_DIRECTION) a = wrap360(360.0f - a);

      if (!angleValid) {
        filteredAngle = a;
        angleValid = true;
      } else {
        filteredAngle = wrap360(filteredAngle + FILTER_ALPHA * wrapDiff(a, filteredAngle));
      }

      float shifted = wrap360(filteredAngle - ZONE_OFFSET_DEG);
      int desired = desiredZone(shifted, currentZone);

      if (currentZone < 0) {
        // first valid reading (boot or after error): adopt zone
        currentZone = desired;
        pendingZone = -1;
        if (PLAY_ON_BOOT) triggerZone(currentZone);
      } else if (desired == currentZone) {
        pendingZone = -1;
      } else if (desired != pendingZone) {
        pendingZone = desired;
        pendingSince = now;
      } else if (now - pendingSince >= DWELL_MS) {
        currentZone = desired;
        pendingZone = -1;
        triggerZone(currentZone);
      }

#if DEBUG_ANGLE
      static uint32_t lastPrint = 0;
      if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("angle=%.1f shifted=%.1f zone=%d\n", filteredAngle, shifted, currentZone);
      }
#endif
    } else {
      if (++i2cFailures >= I2C_FAIL_LIMIT) enterSensorError();
    }
  }

  // ---- audio file problems -> red flash ----
  uint32_t errs = audioErrorCount;
  if (errs != seenAudioErrors) {
    seenAudioErrors = errs;
    ledFlash(COLOR_NOFILE);
  }

  // ---- LEDs ----
  if (now - lastLed >= LED_FRAME_MS) {
    lastLed = now;
    ledRender(now, currentZone, filteredAngle, sensorError);
  }

  delay(1);
}
