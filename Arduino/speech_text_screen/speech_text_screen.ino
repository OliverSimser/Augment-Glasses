// Augment-Glasses: INMP441 speech capture on an ESP32, streamed to an iPhone over BLE.
// Captures 16 kHz, 16-bit mono PCM in 20 ms frames, compresses it 4:1 with IMA ADPCM
// (64 kbps), and notifies it to the phone. The phone transcribes it and writes the
// caption text back, which is printed to Serial until the display is added.
// No WiFi is used.
//
// Board: "ESP32 Dev Module" or "ESP32S3 Dev Module". Serial Monitor at 115200.
// Wiring, INMP441 L/R -> GND, VDD -> 3V3, plus:
//   ESP32     SCK -> GPIO 26, WS -> GPIO 25, SD -> GPIO 33 (same as washer_bluer.ino)
//   ESP32-S3  SCK -> GPIO 4,  WS -> GPIO 5,  SD -> GPIO 6  (same as dryer_and_washer_bluer.ino)
// On the S3, use the USB port marked UART/COM, or enable Tools > USB CDC On Boot to
// get the Serial Monitor on the native USB port.
//
// BLE protocol (the iPhone app in ../../iOS must match):
//   Service  4a8e0001-6c2b-4e5f-9a3d-1c7b2f0e8d10
//   Audio    4a8e0002-...  notify. [seq u8][step index u8][predictor i16 LE][ADPCM nibbles,
//            low nibble first]. Each packet carries the decoder state, so a lost packet
//            only loses its own audio. Frames dropped on the ESP32 skip sequence numbers,
//            so the phone sees them as lost packets and pads them with silence.
//   Caption  4a8e0003-...  write / write-without-response. [flags u8][UTF-8 text].
//            flags bit 0 = text is final. The text is the whole caption to show.
#include <Arduino.h>
#include <math.h>
#include <driver/i2s_std.h>
#include <freertos/queue.h>
#include <BLEDevice.h>
#include <BLEServer.h>
// The classic ESP32 uses the Bluedroid BLE stack. The S3 uses NimBLE on Arduino-ESP32
// core 3.3 and later (Bluedroid on 3.0-3.2); NimBLE adds the subscribe descriptor
// itself and has different callback signatures.
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif

// ---------------- Pins ----------------
#if CONFIG_IDF_TARGET_ESP32
#define I2S_SCK 26
#define I2S_WS 25
#define I2S_SD 33
#elif CONFIG_IDF_TARGET_ESP32S3
// GPIO 26-37 are taken by flash/PSRAM on most S3 modules, so the S3 uses low pins.
#define I2S_SCK 4
#define I2S_WS 5
#define I2S_SD 6
#else
#error "Pins are only defined for the ESP32 and ESP32-S3"
#endif

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

// The index lets the BLE sender see frames the queue dropped, so the phone can
// pad the gap with silence instead of splicing the audio together.
struct AudioFrame {
  uint32_t index;
  int16_t pcm[FRAME_SAMPLES];
};

int32_t rawFrame[FRAME_SAMPLES];
AudioFrame micFrame;      // written only by micTask
AudioFrame overflowFrame;  // micTask's scratch space for discarding the oldest frame
AudioFrame loopFrame;     // written only by loop()
uint32_t micFrameCounter = 0;

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

// ---------------- BLE ----------------
#define BLE_DEVICE_NAME "AugmentGlasses"
#define SERVICE_UUID "4a8e0001-6c2b-4e5f-9a3d-1c7b2f0e8d10"
#define AUDIO_CHAR_UUID "4a8e0002-6c2b-4e5f-9a3d-1c7b2f0e8d10"
#define CAPTION_CHAR_UUID "4a8e0003-6c2b-4e5f-9a3d-1c7b2f0e8d10"

const size_t AUDIO_HEADER_BYTES = 4;
const uint16_t DEFAULT_ATT_MTU = 23;
const uint16_t MAX_ATT_MTU = 517;
const size_t MAX_CAPTION_BYTES = MAX_ATT_MTU - 3 - 1;  // one ATT write, minus the flags byte

BLEServer* bleServer = NULL;
BLECharacteristic* audioChar = NULL;
#if defined(CONFIG_BLUEDROID_ENABLED)
BLE2902* audioCccd = NULL;
#else
volatile bool audioNotifying = false;  // set from NimBLE's onSubscribe
#endif

volatile bool bleConnected = false;
volatile uint16_t peerMtu = DEFAULT_ATT_MTU;

uint8_t audioSeq = 0;
uint8_t audioPacket[MAX_ATT_MTU];
uint32_t packetsThisSecond = 0;
uint32_t notifyFailures = 0;

// Written by the BLE task, read by loop().
portMUX_TYPE captionMux = portMUX_INITIALIZER_UNLOCKED;
char captionIn[MAX_CAPTION_BYTES + 1];
size_t captionInLen = 0;
uint8_t captionInFlags = 0;
volatile bool captionDirty = false;
char captionOut[MAX_CAPTION_BYTES + 1];

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
    micFrame.pcm[i] = (int16_t)lrintf(scaled);
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
    micFrame.index = micFrameCounter++;

    // Never block: on overflow, discard the oldest frame so captions stay current.
    if (xQueueSend(audioQueue, &micFrame, 0) != pdTRUE) {
      xQueueReceive(audioQueue, &overflowFrame, 0);
      xQueueSend(audioQueue, &micFrame, 0);
      droppedFrames = droppedFrames + 1;  // sole writer
    }
  }
}

// ---------------- IMA ADPCM ----------------
// Standard 4-bit IMA codec. The phone's decoder uses the same two tables.
const int16_t ADPCM_STEPS[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
  253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
  1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
  3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487,
  12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};
const int8_t ADPCM_INDEX_ADJUST[16] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };

int32_t adpcmPredictor = 0;
int32_t adpcmIndex = 0;

uint8_t adpcmEncode(int16_t sample) {
  int32_t step = ADPCM_STEPS[adpcmIndex];
  int32_t diff = (int32_t)sample - adpcmPredictor;
  uint8_t code = 0;
  if (diff < 0) {
    code = 8;
    diff = -diff;
  }

  int32_t delta = step >> 3;
  if (diff >= step) {
    code |= 4;
    diff -= step;
    delta += step;
  }
  step >>= 1;
  if (diff >= step) {
    code |= 2;
    diff -= step;
    delta += step;
  }
  step >>= 1;
  if (diff >= step) {
    code |= 1;
    delta += step;
  }

  adpcmPredictor += (code & 8) ? -delta : delta;
  if (adpcmPredictor > 32767) adpcmPredictor = 32767;
  if (adpcmPredictor < -32768) adpcmPredictor = -32768;

  adpcmIndex += ADPCM_INDEX_ADJUST[code];
  if (adpcmIndex < 0) adpcmIndex = 0;
  if (adpcmIndex > 88) adpcmIndex = 88;
  return code;
}

// ---------------- BLE callbacks ----------------
// 15-30 ms interval, inside Apple's accessory guidelines. 50 packets/s needs at
// most two packets per connection event.
const uint16_t CONN_MIN_INTERVAL = 12;  // x 1.25 ms
const uint16_t CONN_MAX_INTERVAL = 24;
const uint16_t CONN_TIMEOUT = 400;  // x 10 ms

void onPhoneConnected() {
  bleConnected = true;
  Serial.println("[BLE] phone connected");
}

void onPhoneDisconnected() {
  bleConnected = false;
  peerMtu = DEFAULT_ATT_MTU;
  // The phone re-subscribes after it reconnects. Bluedroid would otherwise keep
  // the old subscription flag across connections.
#if defined(CONFIG_BLUEDROID_ENABLED)
  if (audioCccd != NULL) audioCccd->setNotifications(false);
#else
  audioNotifying = false;
#endif
  Serial.println("[BLE] phone disconnected, advertising again");
}

void onMtu(uint16_t mtu) {
  peerMtu = mtu;
  Serial.printf("[BLE] MTU %u\n", (unsigned)mtu);
}

class ServerCallbacks : public BLEServerCallbacks {
  using BLEServerCallbacks::onConnect;
  using BLEServerCallbacks::onDisconnect;
  using BLEServerCallbacks::onMtuChanged;

#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) override {
    onPhoneConnected();
    server->requestConnParams(param->connect.remote_bda, CONN_MIN_INTERVAL, CONN_MAX_INTERVAL, 0, CONN_TIMEOUT);
  }
  void onDisconnect(BLEServer*, esp_ble_gatts_cb_param_t*) override {
    onPhoneDisconnected();
  }
  void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
    onMtu(param->mtu.mtu);
  }
#else
  void onConnect(BLEServer* server, ble_gap_conn_desc* desc) override {
    onPhoneConnected();
    server->requestConnParams(desc->conn_handle, CONN_MIN_INTERVAL, CONN_MAX_INTERVAL, 0, CONN_TIMEOUT);
  }
  void onDisconnect(BLEServer*, ble_gap_conn_desc*) override {
    onPhoneDisconnected();
  }
  void onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t mtu) override {
    onMtu(mtu);
  }
#endif
};

class AudioCallbacks : public BLECharacteristicCallbacks {
  // Called synchronously from notify(), i.e. from loop().
  void onStatus(BLECharacteristic*, Status s, uint32_t) override {
    if (s == Status::SUCCESS_NOTIFY) packetsThisSecond++;
    else if (s == Status::ERROR_GATT) notifyFailures++;
  }

#if !defined(CONFIG_BLUEDROID_ENABLED)
  void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t subValue) override {
    audioNotifying = (subValue & 0x0001) != 0;
  }
#endif
};

class CaptionCallbacks : public BLECharacteristicCallbacks {
  using BLECharacteristicCallbacks::onWrite;

  // Runs in the BLE task, so only copy the text out here.
  void onWrite(BLECharacteristic* c) override {
    size_t len = c->getLength();
    if (len < 1) return;
    const uint8_t* data = c->getData();
    size_t textLen = len - 1;
    if (textLen > MAX_CAPTION_BYTES) textLen = MAX_CAPTION_BYTES;

    portENTER_CRITICAL(&captionMux);
    captionInFlags = data[0];
    memcpy(captionIn, data + 1, textLen);
    captionIn[textLen] = '\0';
    captionInLen = textLen;
    captionDirty = true;
    portEXIT_CRITICAL(&captionMux);
  }
};

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(MAX_ATT_MTU);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  bleServer->advertiseOnDisconnect(true);

  BLEService* service = bleServer->createService(SERVICE_UUID);

  audioChar = service->createCharacteristic(AUDIO_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  audioCccd = new BLE2902();  // Bluedroid does not add the subscribe descriptor itself
  audioChar->addDescriptor(audioCccd);
#endif
  audioChar->setCallbacks(new AudioCallbacks());

  BLECharacteristic* captionChar = service->createCharacteristic(
    CAPTION_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  captionChar->setCallbacks(new CaptionCallbacks());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);  // the 128-bit UUID fills the advert, so the name goes here
  BLEDevice::startAdvertising();
  Serial.printf("[BLE] advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

bool audioSubscribed() {
#if defined(CONFIG_BLUEDROID_ENABLED)
  return bleConnected && audioCccd != NULL && audioCccd->getNotifications();
#else
  return bleConnected && audioNotifying;
#endif
}

// Splits a frame into as few packets as the negotiated MTU allows (one per frame
// at iPhone MTUs). Each packet starts with the encoder state for its first sample.
void sendAudioFrame(uint32_t frameIndex, const int16_t* pcm, size_t samples) {
  static bool haveLastFrame = false;
  static uint32_t lastFrameIndex = 0;
  uint32_t skippedFrames = haveLastFrame ? frameIndex - lastFrameIndex - 1 : 0;
  haveLastFrame = true;
  lastFrameIndex = frameIndex;

  if (!audioSubscribed()) return;

  size_t maxPayload = (size_t)peerMtu - 3 - AUDIO_HEADER_BYTES;
  size_t samplesPerPacket = maxPayload * 2;
  if (samplesPerPacket > samples) samplesPerPacket = samples;

  // Skip the sequence numbers the dropped frames would have used; the phone fills
  // the gap with silence. Large gaps are left unpadded by the phone.
  size_t packetsPerFrame = (samples + samplesPerPacket - 1) / samplesPerPacket;
  audioSeq += (uint8_t)(skippedFrames * packetsPerFrame);

  for (size_t offset = 0; offset < samples; offset += samplesPerPacket) {
    size_t n = samples - offset;
    if (n > samplesPerPacket) n = samplesPerPacket;

    audioPacket[0] = audioSeq++;
    audioPacket[1] = (uint8_t)adpcmIndex;
    audioPacket[2] = (uint8_t)(adpcmPredictor & 0xFF);
    audioPacket[3] = (uint8_t)((adpcmPredictor >> 8) & 0xFF);
    size_t len = AUDIO_HEADER_BYTES;

    for (size_t i = 0; i < n; i += 2) {
      uint8_t lo = adpcmEncode(pcm[offset + i]);
      uint8_t hi = (i + 1 < n) ? adpcmEncode(pcm[offset + i + 1]) : 0;
      audioPacket[len++] = lo | (hi << 4);
    }

    audioChar->setValue(audioPacket, len);
    audioChar->notify();
  }
}

// This is where the caption will be drawn on the display.
void showCaptionIfChanged() {
  if (!captionDirty) return;

  uint8_t flags;
  portENTER_CRITICAL(&captionMux);
  memcpy(captionOut, captionIn, captionInLen + 1);
  flags = captionInFlags;
  captionDirty = false;
  portEXIT_CRITICAL(&captionMux);

  Serial.printf("[CAPTION%s] %s\n", (flags & 1) ? " final" : "", captionOut);
}

// ---------------- Consumer ----------------
// Every 20 ms frame of 16 kHz PCM lands here.
void handleAudioFrame(const AudioFrame& frame) {
  const int16_t* pcm = frame.pcm;
  const size_t samples = FRAME_SAMPLES;
  for (size_t i = 0; i < samples; i++) {
    int32_t s = pcm[i];
    levelSumSquares += (double)s * s;
    int32_t a = s < 0 ? -s : s;
    if (a > levelPeak) levelPeak = a;
    if (a >= 32767) clippedThisSecond++;
  }
  levelSamples += samples;
  framesThisSecond++;

  sendAudioFrame(frame.index, pcm, samples);
}

float toDbfs(double amplitude) {
  if (amplitude < 1.0) return -96.0f;
  return 20.0f * log10f((float)(amplitude / 32768.0));
}

void printLevel() {
  double rms = levelSamples ? sqrt(levelSumSquares / levelSamples) : 0.0;
  const char* ble = audioSubscribed() ? "streaming" : (bleConnected ? "connected" : "waiting");
  Serial.printf("[MIC] rms %6.1f dBFS  peak %6.1f dBFS  frames %2u/s  clipped %u  dropped %u  %s"
                "  | ble %s  pkts %2u/s  fail %u\n",
                toDbfs(rms), toDbfs(levelPeak), (unsigned)framesThisSecond,
                (unsigned)clippedThisSecond, (unsigned)droppedFrames,
                micHealthy ? "ok" : "NO SIGNAL",
                ble, (unsigned)packetsThisSecond, (unsigned)notifyFailures);
  levelSumSquares = 0.0;
  levelPeak = 0;
  levelSamples = 0;
  framesThisSecond = 0;
  clippedThisSecond = 0;
  packetsThisSecond = 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("Augment-Glasses mic: %u Hz, 16-bit mono, %u ms frames, gain %+.0f dB\n",
                (unsigned)SAMPLE_RATE, (unsigned)FRAME_MS, MIC_GAIN_DB);

  gainLinear = powf(10.0f, MIC_GAIN_DB / 20.0f);

  audioQueue = xQueueCreate(QUEUE_FRAMES, sizeof(AudioFrame));
  if (audioQueue == NULL) {
    Serial.println("[MIC] audio queue allocation failed, halting");
    for (;;) delay(1000);
  }

  if (setupMic() != ESP_OK) {
    micHealthy = false;
    Serial.println("[MIC] initial setup failed, mic task will keep retrying");
  }

  setupBle();

  // Core 1 with loop(); core 0 is left for the BLE stack.
  xTaskCreatePinnedToCore(micTask, "mic", 4096, NULL, 5, NULL, 1);
  lastLevelPrint = millis();
}

void loop() {
  // Frames arrive every 20 ms, so while audio flows this loop only exits for the
  // level print. Captions are checked per frame so they show within 20 ms.
  while (xQueueReceive(audioQueue, &loopFrame, pdMS_TO_TICKS(50)) == pdTRUE) {
    handleAudioFrame(loopFrame);
    showCaptionIfChanged();
    if (millis() - lastLevelPrint >= 1000) break;
  }

  showCaptionIfChanged();

  if (millis() - lastLevelPrint >= 1000) {
    lastLevelPrint = millis();
    printLevel();
  }
}

