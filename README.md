# Augment-Glasses
See-through captioning glasses using virtual-image optics to display real-time speech-to-text, helping deaf and hard of hearing users follow conversations

## Arduino sketch

Open [speech_text_screen.ino](Arduino/speech_text_screen/speech_text_screen.ino)
in the Arduino IDE and select **ESP32 Dev Module**. The sketch is in a matching
folder as required by the Arduino IDE.

Wiring (INMP441 I2S microphone):

| INMP441 | ESP32    |
|---------|----------|
| SCK     | GPIO 26  |
| WS      | GPIO 25  |
| SD      | GPIO 33  |
| L/R     | GND      |
| VDD     | 3V3      |

The sketch captures 16 kHz, 16-bit mono PCM in 20 ms frames, compresses it 4:1 with
IMA ADPCM (64 kbps), and streams it to the iPhone app over BLE. Once a second, the
Serial Monitor (115200 baud) shows the RMS and peak level in dBFS, frames per second
(should be 50), clipped samples, dropped frames, and the BLE state (`waiting`,
`connected`, or `streaming`) with packets sent per second. Adjust `MIC_GAIN_DB` until
speech peaks reach about -12 to -6 dBFS without clipping (24 dB tested well).

Captions sent back by the phone are printed as `[CAPTION] ...` lines. Display output
is not implemented yet.

## How it works

No WiFi is used. Everything goes over BLE:

1. The ESP32 advertises as `AugmentGlasses` and streams compressed audio.
2. The iPhone app connects, decodes the audio, and transcribes it with Apple's
   on-device speech-to-text (`SpeechAnalyzer`, iOS 26). No audio leaves the phone,
   and it works without cell data after the one-time model download.
3. The app writes the latest caption text back to the ESP32 about 10 times a second.

## iPhone app

The source is in [iOS/AugmentGlasses](iOS/AugmentGlasses). It needs a Mac with
Xcode 26 and an iPhone on iOS 26. A free Apple ID works for testing, but apps it
installs expire after 7 days and must be reinstalled from Xcode.

### One-time setup in Xcode

1. **File > New > Project > iOS > App.** Name it `AugmentGlasses`, interface
   **SwiftUI**, language **Swift**, no storage, no tests.
2. In the project navigator, delete the generated `ContentView.swift` and
   `AugmentGlassesApp.swift` (Move to Trash).
3. Drag the six `.swift` files from `iOS/AugmentGlasses` into the project's
   `AugmentGlasses` group. Check **Copy items if needed** and the app target.
4. Select the project > **AugmentGlasses** target > **General**, and set
   **Minimum Deployments** to **iOS 26.0**.
5. On the **Info** tab, add these two keys under **Custom iOS Target Properties**
   (click **+** on any row):
   - `Privacy - Bluetooth Always Usage Description`: `Connects to your captioning glasses.`
   - `Privacy - Speech Recognition Usage Description`: `Turns the glasses' audio into captions.`
6. On **Signing & Capabilities**, choose your **Team** (sign in with your Apple ID).
7. Plug in the iPhone, select it as the run destination, and press **Run**. The
   first time, enable **Settings > Privacy & Security > Developer Mode** on the
   iPhone, and trust your developer certificate under **Settings > General > VPN &
   Device Management**.

### First run

1. Power the ESP32 and open its Serial Monitor.
2. Open the app and allow Bluetooth and Speech Recognition. On first launch it
   downloads the speech model for your language (needs data or WiFi once).
3. The app shows **Receiving audio** with about 50 packets/s and 0 lost. The Serial
   Monitor shows `ble streaming  pkts 50/s  fail 0`.
4. Speak near the mic. Captions appear in the app and as `[CAPTION]` lines in the
   Serial Monitor.

Keep the app open on screen while testing; it keeps the screen awake. Running in the
background needs **Signing & Capabilities > + Capability > Background Modes > Uses
Bluetooth LE accessories**, and has not been tested yet.
