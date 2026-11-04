/*
 * Knob Sound Device  -  ESP8266 (Wemos/LOLIN D1 mini Lite)  -  POTENTIOMETER version
 * ----------------------------------------------------------------------------------
 * Potentiometer on A0 + MAX98357A + WS2812 strip
 *
 * Same behaviour as the other builds:
 * - The pot's travel is split into NUM_ZONES equal zones. Entering a zone plays
 *   that zone's sound (/z0.wav, /z1.wav ...). Sounds fade in/out, volume is capped.
 * - LED strip: zone-colour glow, pointer for the knob position, flash on zone
 *   entry, breathing while a sound plays.
 *
 * IMPORTANT: a standard potentiometer only turns about 270-300 degrees and has
 * mechanical end stops. It CANNOT rotate a full 360 degrees. The zones are spread
 * over the pot's usable travel (ADC_MIN..ADC_MAX), so with 8 zones each zone is
 * about 1/8 of that travel (roughly 34 degrees on a 270 degree pot).
 * The position is ABSOLUTE: the same knob position always gives the same sound,
 * even right after power-up.
 *
 * REQUIREMENTS
 *   - ESP8266 Arduino core 3.x   (board: "LOLIN(WEMOS) D1 mini Lite")
 *   - Library: "Adafruit NeoPixel"
 *   - Flash layout with a filesystem, e.g. "1MB (FS:512KB ...)" on a 1 MB board
 *   - WAV files uploaded to LittleFS
 *
 * AUDIO FILE FORMAT (all files must match)
 *   WAV, PCM, 16-bit, mono or stereo, sample rate = SAMPLE_RATE below.
 *     ffmpeg -i input.mp3 -ar 16000 -ac 1 -c:a pcm_s16le z0.wav
 *
 * WIRING  (ESP8266 hardware I2S pins are FIXED)
 *   MAX98357A  DIN  -> RX  (GPIO3)
 *              BCLK -> D8  (GPIO15)
 *              LRC  -> D4  (GPIO2)
 *              VIN  -> 5V, GND -> GND   (leave SD and GAIN at breakout defaults)
 *   Potentiometer (10k linear, "B10K" recommended)
 *              one outer pin -> 3V3   (NOT 5V: A0 tolerates at most ~3.2 V)
 *              other outer pin -> GND
 *              middle pin (wiper) -> A0
 *              Optional: 100 nF capacitor from the wiper (A0) to GND to reduce noise
 *   WS2812     DIN  -> D5  (GPIO14) via ~330 ohm, 5V from a proper supply,
 *              COMMON GROUND with the board, 1000 uF cap across 5V/GND.
 *
 *   Notes:
 *   - RX is used by I2S, so the serial port is TX-only (debug output works,
 *     typing into the Serial Monitor does not).
 *   - If uploading fails, unplug the MAX98357A DIN wire from RX while flashing.
 *   - The D1 mini's A0 pin has an on-board voltage divider for 0-3.2 V input and
 *     reads 0..1023. Use DEBUG_ADC to check your real min/max values.
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <i2s.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

// ============================================================================
//  CONFIGURATION
// ============================================================================

// ---- Pins (GPIO numbers) ---------------------------------------------------
constexpr int PIN_LED = 14;   // D5
// Potentiometer wiper -> A0. I2S is fixed by hardware:
// DATA=GPIO3 (RX), BCLK=GPIO15 (D8), LRC=GPIO2 (D4)

// ---- Potentiometer ---------------------------------------------------------
// Raw ADC values (0..1023) at the two ends of the usable travel. Defaults leave
// a small margin so the end stops do not flicker. Set DEBUG_ADC to 1, turn the
// knob to each end and adjust these two numbers to match.
constexpr int   ADC_MIN          = 4;
constexpr int   ADC_MAX          = 1019;
constexpr bool  INVERT_DIRECTION = false;   // true if turning clockwise should go to lower zones
constexpr float FILTER_ALPHA     = 0.20f;   // 0..1, lower = smoother
constexpr int   ADC_SAMPLES      = 4;       // readings averaged per update

// ---- Knob zones ------------------------------------------------------------
constexpr int   NUM_ZONES        = 8;       // equal zones across the pot travel
constexpr float HYSTERESIS_FRAC  = 0.012f;  // dead band at zone edges (fraction of full travel)
constexpr uint32_t DWELL_MS      = 40;      // position must stay in new zone this long
constexpr bool  PLAY_ON_BOOT     = false;   // play the starting zone's sound at power-up?

constexpr uint32_t SENSE_INTERVAL_MS = 5;   // how often the pot is read

// ---- Audio -----------------------------------------------------------------
constexpr uint32_t SAMPLE_RATE   = 16000;  // all WAV files must use this rate
constexpr float    MASTER_VOLUME = 0.40f;  // 0.0 .. 1.0  (also the hard cap)
constexpr uint32_t FADE_MS       = 60;     // fade in / fade out time
constexpr uint32_t FLUSH_MS      = 50;     // silence written when playback ends
constexpr int      CHUNK_FRAMES  = 128;    // frames read from flash at a time

// ---- LEDs ------------------------------------------------------------------
constexpr int      LED_COUNT          = 16;
constexpr uint8_t  LED_MAX_BRIGHTNESS = 60;   // 0..255 global cap
constexpr uint32_t LED_FRAME_MS       = 25;   // 40 fps
constexpr uint32_t FLASH_MS           = 300;  // zone-enter flash length

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
static_assert(ADC_MAX > ADC_MIN, "ADC_MAX must be greater than ADC_MIN");

constexpr RGB COLOR_READY  = {  0, 200,  60};
constexpr RGB COLOR_NOFILE = {200,   0,  40};  // sound file missing/invalid
constexpr RGB COLOR_IDLE   = { 60,  60,  60};

// Set to 1 to print ADC / position / zone to Serial (115200, TX only)
#define DEBUG_ADC 0

// ============================================================================
//  GLOBALS
// ============================================================================

Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

// ============================================================================
//  POTENTIOMETER
// ============================================================================

float potFiltered = 0.0f;
bool  potValid    = false;
float potRaw      = 0.0f;   // last averaged raw reading (for debug)

// Returns the knob position as a fraction 0.0 .. 1.0 of the usable travel.
float readPotFraction() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) sum += (uint32_t)analogRead(A0);
  potRaw = (float)sum / (float)ADC_SAMPLES;

  if (!potValid) {
    potFiltered = potRaw;
    potValid = true;
  } else {
    potFiltered += FILTER_ALPHA * (potRaw - potFiltered);
  }

  float f = (potFiltered - (float)ADC_MIN) / (float)(ADC_MAX - ADC_MIN);
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  if (INVERT_DIRECTION) f = 1.0f - f;
  return f;
}

// Zone for a position fraction, with hysteresis around the currently held zone.
int desiredZone(float f, int heldZone) {
  const float W = 1.0f / NUM_ZONES;
  if (heldZone >= 0) {
    float lo = (float)heldZone * W - HYSTERESIS_FRAC;
    float hi = (float)(heldZone + 1) * W + HYSTERESIS_FRAC;
    if (f >= lo && f <= hi) return heldZone;  // still inside held zone (plus dead band)
  }
  int z = (int)(f * (float)NUM_ZONES);
  if (z < 0) z = 0;
  if (z >= NUM_ZONES) z = NUM_ZONES - 1;
  return z;
}

// ============================================================================
//  AUDIO
//  Cooperative player: audioPump() is called many times per loop() and feeds
//  the hardware I2S FIFO without blocking. Gain maths is integer (Q15) because
//  the ESP8266 has no FPU.
// ============================================================================

enum PlayState { PS_IDLE, PS_PLAYING, PS_FADING_OUT, PS_FLUSH };

constexpr int32_t  GAIN_ONE       = 32768;                              // Q15 "1.0"
constexpr int32_t  VOLUME_Q15     = (int32_t)(MASTER_VOLUME * 32768.0f);
constexpr uint32_t FADE_FRAMES    = (SAMPLE_RATE * FADE_MS) / 1000;
constexpr uint32_t FLUSH_FRAMES   = (SAMPLE_RATE * FLUSH_MS) / 1000;

File      wavFile;
uint16_t  wavChannels  = 1;
uint32_t  fileRemaining = 0;      // data bytes not yet read from flash
uint32_t  framesLeft   = 0;       // frames not yet output
uint32_t  fadeOutFrames = 0;      // length of the end-of-file fade for this file

int16_t   inBuf[CHUNK_FRAMES * 2];
int       inFrames = 0;
int       inPos    = 0;

PlayState st       = PS_IDLE;
int       pendingZone = -1;
int32_t   gain     = 0;           // Q15
int32_t   target   = GAIN_ONE;    // Q15
int32_t   gainStep = 1;           // Q15 per frame
uint32_t  flushLeft = 0;

bool      audioPlaying    = false;
uint32_t  audioErrorCount = 0;

bool      havePending   = false;
uint32_t  pendingSample = 0;

void closeWav() {
  if (wavFile) wavFile.close();
  fileRemaining = 0;
  inFrames = 0;
  inPos = 0;
}

// Open /z<zone>.wav, validate the header, position at the PCM data.
bool startWav(int zone) {
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
  uint32_t dataBytes = 0;

  while (wavFile.available()) {
    uint8_t ch[8];
    if (wavFile.read(ch, 8) != 8) break;
    uint32_t sz = (uint32_t)ch[4] | ((uint32_t)ch[5] << 8) | ((uint32_t)ch[6] << 16) | ((uint32_t)ch[7] << 24);

    if (memcmp(ch, "fmt ", 4) == 0) {
      if (sz < 16) break;
      uint8_t fm[16];
      if (wavFile.read(fm, 16) != 16) break;
      uint16_t format   = fm[0] | (fm[1] << 8);
      uint16_t channels = fm[2] | (fm[3] << 8);
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
      dataBytes = (sz > avail) ? avail : sz;
      dataFound = true;
      break;
    } else {
      wavFile.seek(wavFile.position() + sz + (sz & 1));  // skip LIST etc.
    }
  }

  const uint32_t frameBytes = (uint32_t)wavChannels * 2;
  if (!fmtOk || !dataFound || dataBytes < frameBytes) {
    Serial.printf("[audio] %s has no usable data\n", path);
    closeWav();
    return false;
  }

  fileRemaining = dataBytes;
  framesLeft    = dataBytes / frameBytes;
  fadeOutFrames = FADE_FRAMES;
  if (fadeOutFrames > framesLeft / 2) fadeOutFrames = framesLeft / 2;  // very short sounds

  uint32_t ff = FADE_FRAMES > 0 ? FADE_FRAMES : 1;
  gainStep = GAIN_ONE / (int32_t)ff;
  if (gainStep < 1) gainStep = 1;
  gain   = 0;
  target = GAIN_ONE;
  inFrames = 0;
  inPos = 0;
  return true;
}

// Called when the current file is done (or faded out). Starts a pending
// sound if there is one, otherwise flushes silence and goes idle.
void finishCurrent() {
  closeWav();
  if (pendingZone >= 0) {
    int z = pendingZone;
    pendingZone = -1;
    if (startWav(z)) {
      st = PS_PLAYING;
      audioPlaying = true;
      return;
    }
    audioErrorCount++;
  }
  st = PS_FLUSH;
  flushLeft = FLUSH_FRAMES;
  audioPlaying = false;
}

// Produce the next stereo sample (low 16 bits = left, high 16 bits = right).
// Returns false if there is nothing to output.
bool audioNextSample(uint32_t &out) {
  if (st == PS_IDLE) return false;

  if (st == PS_FLUSH) {
    if (flushLeft > 0) {
      flushLeft--;
      out = 0;
      return true;
    }
    st = PS_IDLE;
    return false;
  }

  // PS_PLAYING or PS_FADING_OUT
  const uint32_t frameBytes = (uint32_t)wavChannels * 2;

  if (inPos >= inFrames) {
    if (fileRemaining == 0) {
      finishCurrent();
      out = 0;
      return true;
    }
    uint32_t want = (uint32_t)CHUNK_FRAMES * frameBytes;
    if (want > fileRemaining) want = fileRemaining;
    size_t got = wavFile.read((uint8_t *)inBuf, want);
    got -= got % frameBytes;
    if (got == 0) {
      fileRemaining = 0;
      finishCurrent();
      out = 0;
      return true;
    }
    fileRemaining -= (uint32_t)got;
    inFrames = (int)(got / frameBytes);
    inPos = 0;
  }

  int32_t s;
  if (wavChannels == 1) {
    s = inBuf[inPos];
  } else {
    s = ((int32_t)inBuf[2 * inPos] + (int32_t)inBuf[2 * inPos + 1]) / 2;
  }
  inPos++;

  // begin the end-of-file fade shortly before the last frame
  if (st == PS_PLAYING && framesLeft <= fadeOutFrames) target = 0;
  if (framesLeft > 0) framesLeft--;

  if (gain < target)      { gain += gainStep; if (gain > target) gain = target; }
  else if (gain > target) { gain -= gainStep; if (gain < target) gain = target; }

  if (st == PS_FADING_OUT && gain == 0) {
    finishCurrent();
    out = 0;
    return true;
  }

  int32_t v = (s * gain) >> 15;        // apply fade gain
  v = (v * VOLUME_Q15) >> 15;          // apply master volume
  if (v > 32767)  v = 32767;
  if (v < -32768) v = -32768;
  uint16_t u = (uint16_t)(int16_t)v;
  out = ((uint32_t)u << 16) | u;
  return true;
}

// Feed the I2S hardware until its buffer is full. Call this OFTEN.
void audioPump() {
  for (;;) {
    if (!havePending) {
      if (!audioNextSample(pendingSample)) return;
      havePending = true;
    }
    if (!i2s_write_sample_nb(pendingSample)) return;  // buffer full
    havePending = false;
  }
}

void audioRequest(int zone) {
  if (st == PS_IDLE || st == PS_FLUSH) {
    // output is silent right now, safe to open the file immediately
    pendingZone = -1;
    if (startWav(zone)) {
      st = PS_PLAYING;
      audioPlaying = true;
    } else {
      audioErrorCount++;
    }
  } else {
    pendingZone = zone;  // latest request wins; fade current sound out first
    st = PS_FADING_OUT;
    target = 0;
  }
}

// ============================================================================
//  LEDs
// ============================================================================

RGB      flashColor  = COLOR_READY;
uint32_t flashStart  = 0;
bool     flashActive = false;

void ledFlash(const RGB &c) {
  flashColor  = c;
  flashStart  = millis();
  flashActive = true;
}

static inline uint8_t scale8f(uint8_t v, float k) {
  float x = (float)v * k;
  if (x < 0.0f)   x = 0.0f;
  if (x > 255.0f) x = 255.0f;
  return (uint8_t)x;
}

// posFrac: knob position 0.0 .. 1.0 across the whole strip
void ledRender(uint32_t now, int zone, float posFrac) {
  RGB c = (zone >= 0) ? ZONE_COLORS[zone] : COLOR_IDLE;

  // base glow: steady when silent, slow breathing while a sound plays
  float k = 0.12f;
  if (audioPlaying) k = 0.20f + 0.10f * sinf((float)now * 0.004f);

  uint8_t fr[LED_COUNT], fg[LED_COUNT], fb[LED_COUNT];
  for (int i = 0; i < LED_COUNT; i++) {
    fr[i] = scale8f(c.r, k);
    fg[i] = scale8f(c.g, k);
    fb[i] = scale8f(c.b, k);
  }

  // pointer showing the knob position along the strip
  int p = (int)(posFrac * (float)(LED_COUNT - 1) + 0.5f);
  if (p < 0) p = 0;
  if (p >= LED_COUNT) p = LED_COUNT - 1;
  fr[p] = c.r; fg[p] = c.g; fb[p] = c.b;

  // event flash (zone entered, file error)
  if (flashActive) {
    uint32_t age = now - flashStart;
    if (age >= FLASH_MS) {
      flashActive = false;
    } else {
      float f = (1.0f - (float)age / (float)FLASH_MS) * 0.9f;
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

float    posFrac         = 0.0f;
int      currentZone     = -1;
int      pendingZoneCand = -1;
uint32_t pendingSince    = 0;
uint32_t lastSense       = 0;
uint32_t lastLed         = 0;
uint32_t seenAudioErrors = 0;

void triggerZone(int zone) {
  Serial.printf("[zone] -> %d\n", zone);
  audioRequest(zone);
  ledFlash(ZONE_COLORS[zone]);
}

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  // RX (GPIO3) is needed for I2S data, so use a TX-only serial port
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(100);
  Serial.println("\nKnob Sound Device (ESP8266, potentiometer) starting");

  // Radio off: saves power and removes ADC noise / timing jitter
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // LEDs
  strip.begin();
  strip.setBrightness(LED_MAX_BRIGHTNESS);
  strip.clear();
  strip.show();

  // Filesystem
  if (!LittleFS.begin()) {
    Serial.println("[fs] LittleFS mount failed (flash layout needs a filesystem, files must be uploaded)");
    ledFlash(COLOR_NOFILE);
  } else {
    for (int z = 0; z < NUM_ZONES; z++) {
      char path[24];
      snprintf(path, sizeof(path), "/z%d.wav", z);
      if (!LittleFS.exists(path)) Serial.printf("[fs] WARNING: %s not found\n", path);
    }
  }

  // Hardware I2S (16-bit stereo, same sample on both channels)
  i2s_begin();
  i2s_set_rate(SAMPLE_RATE);

  // Let the filter settle on the real pot position before the first zone decision
  for (int i = 0; i < 20; i++) {
    posFrac = readPotFraction();
    delay(2);
  }

  ledFlash(COLOR_READY);
  Serial.println("Ready");
}

// ============================================================================
//  LOOP
// ============================================================================

void loop() {
  audioPump();
  uint32_t now = millis();

  // ---- pot position -> zone ----
  if (now - lastSense >= SENSE_INTERVAL_MS) {
    lastSense = now;

    posFrac = readPotFraction();
    int desired = desiredZone(posFrac, currentZone);

    if (currentZone < 0) {
      // first reading: adopt the zone
      currentZone = desired;
      pendingZoneCand = -1;
      if (PLAY_ON_BOOT) triggerZone(currentZone);
    } else if (desired == currentZone) {
      pendingZoneCand = -1;
    } else if (desired != pendingZoneCand) {
      pendingZoneCand = desired;
      pendingSince = now;
    } else if (now - pendingSince >= DWELL_MS) {
      currentZone = desired;
      pendingZoneCand = -1;
      triggerZone(currentZone);
    }

#if DEBUG_ADC
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 200) {
      lastPrint = now;
      Serial.printf("adc=%.1f filt=%.1f pos=%.3f zone=%d\n", potRaw, potFiltered, posFrac, currentZone);
    }
#endif
    audioPump();
  }

  // ---- audio file problems -> red flash ----
  if (audioErrorCount != seenAudioErrors) {
    seenAudioErrors = audioErrorCount;
    ledFlash(COLOR_NOFILE);
  }

  // ---- LEDs ----
  if (now - lastLed >= LED_FRAME_MS) {
    lastLed = now;
    ledRender(now, currentZone, posFrac);
    audioPump();
  }
}
