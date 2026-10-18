// Augment-Glasses: INMP441 speech capture on an ESP32-S3.
// Captures 16 kHz, 16-bit mono PCM (the format speech-to-text engines expect) in
// 20 ms frames and queues them for the BLE link to the phone. BLE and the display
// are not added yet; for now loop() drains the queue and logs the input level.
//
// Board: "ESP32S3 Dev Module". Serial Monitor at 115200.
// Wiring (same as the washer node):
//   INMP441  SCK -> GPIO 4, WS -> GPIO 5, SD -> GPIO 6, L/R -> GND, VDD -> 3V3
#include <Arduino.h>
#include <math.h>
#include <driver/i2s_std.h>
#include <freertos/queue.h>

#if !CONFIG_IDF_TARGET_ESP32S3
#error "Pins are for the ESP32-S3 (GPIO 6 is a flash pin on the original ESP32)"
#endif

// ---------------- Pins ----------------
#define I2S_SCK 4
#define I2S_WS 5
#define I2S_SD 6

// ---------------- Audio format ----------------
const uint32_t SAMPLE_RATE = 16000;  // speech content is below 8 kHz; STT models run at 16 kHz
const uint32_t FRAME_MS = 20;
const size_t FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS / 1000;  // 320 samples = 640 bytes
const size_t QUEUE_FRAMES = 25;                              // 500 ms of slack for BLE stalls

// The INMP441 is quiet: conversation at 1 m (~60 dB SPL) lands near -60 dBFS at
// unity gain. Tune with the level log so speech peaks sit around -12 to -6 dBFS.
const float MIC_GAIN_DB = 24.0f;

// One-pole DC blocker. The INMP441 has a DC offset, and 0.98 also cuts rumble
// below ~50 Hz, which carries no speech.
const float DC_BLOCK_R = 0.98f;

// The mic outputs garbage for 2^18 SCK cycles after clocking starts (~256 ms here).
const uint32_t MIC_STARTUP_MS = 300;

// ---------------- Health ----------------
const int MAX_I2S_FAILURES = 20;
int consecutiveI2sFailures = 0;

// A live INMP441 always has a noise floor, so bit-identical samples mean there is
// no signal path. i2s reads still succeed with zeros when the mic is unpowered or
// unwired, so this is the only way that fault becomes visible.
const int MAX_FLAT_FRAMES = 250;  // 5 s of 20 ms frames
int flatFrameCount = 0;
volatile bool micHealthy = true;

// ---------------- State ----------------
i2s_chan_handle_t rxChan = NULL;
QueueHandle_t audioQueue = NULL;
volatile uint32_t droppedFrames = 0;

int32_t rawFrame[FRAME_SAMPLES];
int16_t micPcm[FRAME_SAMPLES];   // written only by micTask
int16_t loopPcm[FRAME_SAMPLES];  // written only by loop()

float dcPrevIn = 0.0f;
float dcPrevOut = 0.0f;
float gainLinear = 1.0f;

// Level stats, accumulated in loop() and printed once a second.
double levelSumSquares = 0.0;
int32_t levelPeak = 0;
uint32_t levelSamples = 0;
uint32_t framesThisSecond = 0;
uint32_t clippedThisSecond = 0;
unsigned long lastLevelPrint = 0;

// ---------------- Microphone ----------------
esp_err_t setupMic() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = FRAME_SAMPLES;

  esp_err_t r = i2s_new_channel(&chanCfg, NULL, &rxChan);
  if (r != ESP_OK) {
    Serial.printf("[I2S] new_channel failed: %d\n", r);
    return r;
  }

  // INMP441: Philips I2S, 24-bit data left-justified in a 32-bit slot, L/R -> GND = left.
  i2s_std_config_t stdCfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,  // the INMP441 has no MCLK input; keep GPIO 0 (BOOT) free
      .bclk = (gpio_num_t)I2S_SCK,
      .ws = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_SD,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  stdCfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  r = i2s_channel_init_std_mode(rxChan, &stdCfg);
  if (r != ESP_OK) {
    Serial.printf("[I2S] init_std_mode failed: %d\n", r);
    i2s_del_channel(rxChan);
    rxChan = NULL;
    return r;
  }
  r = i2s_channel_enable(rxChan);
  if (r != ESP_OK) {
    Serial.printf("[I2S] channel_enable failed: %d\n", r);
    i2s_del_channel(rxChan);
    rxChan = NULL;
    return r;
  }

  dcPrevIn = 0.0f;
  dcPrevOut = 0.0f;
  vTaskDelay(pdMS_TO_TICKS(MIC_STARTUP_MS));
  return ESP_OK;
}

void teardownMic() {
  if (rxChan == NULL) return;
  i2s_channel_disable(rxChan);
  i2s_del_channel(rxChan);
  rxChan = NULL;
}

void restartMic() {
  teardownMic();
  vTaskDelay(pdMS_TO_TICKS(100));
  setupMic();  // logs its own failure; a dead mic cannot be fixed in software
}

void updateMicHealth() {
  bool flat = true;
  for (size_t i = 1; i < FRAME_SAMPLES; i++) {
    if (rawFrame[i] != rawFrame[0]) {
      flat = false;
      break;
    }
  }
  if (flat) {
    if (flatFrameCount < MAX_FLAT_FRAMES) flatFrameCount++;
  } else {
    flatFrameCount = 0;
  }
  if ((flatFrameCount >= MAX_FLAT_FRAMES) == micHealthy) {  // report the edge only
    micHealthy = !micHealthy;
    Serial.printf("[MIC] %s\n", micHealthy ? "signal restored" : "NO SIGNAL: samples flat, mic dead or unwired");
  }
}

// 32-bit I2S slot -> 16-bit PCM with DC removal and gain.
void convertFrame() {
  for (size_t i = 0; i < FRAME_SAMPLES; i++) {
    float in = (float)(rawFrame[i] >> 8);  // 24-bit signed sample
    float out = in - dcPrevIn + DC_BLOCK_R * dcPrevOut;
    dcPrevIn = in;
    dcPrevOut = out;

    float scaled = out * gainLinear / 256.0f;  // /256 = 24-bit -> 16-bit at unity gain
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    micPcm[i] = (int16_t)lrintf(scaled);
  }
}

// Owns the I2S peripheral. Kept separate from loop() so a slow BLE send can never
// stall the mic and overflow the DMA buffers.
void micTask(void*) {
  for (;;) {
    if (rxChan == NULL) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // setup failed earlier; retry slowly, not in a tight loop
      restartMic();
      continue;
    }

    size_t bytesRead = 0;
    esp_err_t r = i2s_channel_read(rxChan, rawFrame, sizeof(rawFrame), &bytesRead, 500);

    // A short read is a failure too: padding it would put clicks into the audio.
    if (r != ESP_OK || bytesRead != sizeof(rawFrame)) {
      if (++consecutiveI2sFailures >= MAX_I2S_FAILURES) {
        Serial.printf("[I2S] Stream dead (err=%d, got %u/%u). Reinstalling...\n",
                      r, (unsigned)bytesRead, (unsigned)sizeof(rawFrame));
        restartMic();
        consecutiveI2sFailures = 0;
      }
      continue;
    }
    consecutiveI2sFailures = 0;

    updateMicHealth();
    convertFrame();

    // Drop the newest frame rather than block: falling behind real time is worse
    // for live captions than a 20 ms gap.
    if (xQueueSend(audioQueue, micPcm, 0) != pdTRUE) droppedFrames = droppedFrames + 1;  // sole writer
  }
}

// ---------------- Consumer ----------------
// Every 20 ms frame of 16 kHz PCM lands here. This is where the BLE notify to the
// phone goes; for now it only feeds the level meter.
void handleAudioFrame(const int16_t* pcm, size_t samples) {
  for (size_t i = 0; i < samples; i++) {
    int32_t s = pcm[i];
    levelSumSquares += (double)s * s;
    int32_t a = s < 0 ? -s : s;
    if (a > levelPeak) levelPeak = a;
    if (a >= 32767) clippedThisSecond++;
  }
  levelSamples += samples;
  framesThisSecond++;
}

float toDbfs(double amplitude) {
  if (amplitude < 1.0) return -96.0f;
  return 20.0f * log10f((float)(amplitude / 32768.0));
}

void printLevel() {
  double rms = levelSamples ? sqrt(levelSumSquares / levelSamples) : 0.0;
  Serial.printf("[MIC] rms %6.1f dBFS  peak %6.1f dBFS  frames %2u/s  clipped %u  dropped %u  %s\n",
                toDbfs(rms), toDbfs(levelPeak), (unsigned)framesThisSecond,
                (unsigned)clippedThisSecond, (unsigned)droppedFrames,
                micHealthy ? "ok" : "NO SIGNAL");
  levelSumSquares = 0.0;
  levelPeak = 0;
  levelSamples = 0;
  framesThisSecond = 0;
  clippedThisSecond = 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("Augment-Glasses mic: %u Hz, 16-bit mono, %u ms frames, gain %+.0f dB\n",
                (unsigned)SAMPLE_RATE, (unsigned)FRAME_MS, MIC_GAIN_DB);

  gainLinear = powf(10.0f, MIC_GAIN_DB / 20.0f);

  audioQueue = xQueueCreate(QUEUE_FRAMES, sizeof(micPcm));
  if (audioQueue == NULL) {
    Serial.println("[MIC] audio queue allocation failed, halting");
    for (;;) delay(1000);
  }

  if (setupMic() != ESP_OK) {
    micHealthy = false;
    Serial.println("[MIC] initial setup failed, mic task will keep retrying");
  }

  // Core 1 with loop(); core 0 is left for the BLE stack.
  xTaskCreatePinnedToCore(micTask, "mic", 4096, NULL, 5, NULL, 1);
  lastLevelPrint = millis();
}

void loop() {
  while (xQueueReceive(audioQueue, loopPcm, pdMS_TO_TICKS(50)) == pdTRUE) {
    handleAudioFrame(loopPcm, FRAME_SAMPLES);
    if (millis() - lastLevelPrint >= 1000) break;
  }

  if (millis() - lastLevelPrint >= 1000) {
    lastLevelPrint = millis();
    printLevel();
  }
}
