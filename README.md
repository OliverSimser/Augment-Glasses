# Augment-Glasses
See-through captioning glasses using virtual-image optics to display real-time speech-to-text, helping deaf and hard of hearing users follow conversations

## Arduino sketch

Open [speech_text_screen.ino](Arduino/speech_text_screen/speech_text_screen.ino)
in the Arduino IDE and select **ESP32 Dev Module** or **ESP32S3 Dev Module**. It
needs the Arduino-ESP32 core 3.x (3.3 or later recommended); core 2.x will not compile
it. The sketch is in a matching folder as required by the Arduino IDE.

Wiring (INMP441 I2S microphone):

| INMP441 | ESP32    | ESP32-S3 |
|---------|----------|----------|
| SCK     | GPIO 26  | GPIO 4   |
| WS      | GPIO 25  | GPIO 5   |
| SD      | GPIO 33  | GPIO 6   |
| L/R     | GND      | GND      |
| VDD     | 3V3      | 3V3      |

On the S3, use the USB port marked UART/COM, or enable **Tools > USB CDC On Boot**
to get the Serial Monitor on the native USB port.

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

1. **File > New > Project**, pick the **iOS** tab at the top (not Multiplatform or
   macOS), then **App**. Name it `AugmentGlasses`, interface **SwiftUI**, language
   **Swift**, no storage, no tests.
2. In the project navigator, delete the generated `ContentView.swift` and
   `AugmentGlassesApp.swift` (Move to Trash). Do this before step 3; otherwise Xcode
   names the copies `ContentView 2.swift` and you end up with two of each.
3. Drag the six `.swift` files from `iOS/AugmentGlasses` into the project's
   `AugmentGlasses` group. Check **Copy items if needed** and the app target.
4. Select the project > **AugmentGlasses** target > **General**, and set
   **Minimum Deployments** to **iOS 26.0**. Under **Supported Destinations**, make
   sure **iPhone** is listed (add it with **+** if the project was made as a Mac or
   Multiplatform app).
5. On the **Info** tab, add these two keys under **Custom iOS Target Properties**
   (hover over any row, click **+**, start typing the key name, pick it from the list,
   then double-click the **Value** column and type the text):
   - `Privacy - Bluetooth Always Usage Description`: `Connects to your captioning glasses.`
   - `Privacy - Speech Recognition Usage Description`: `Turns the glasses' audio into captions.`

   This is done in Xcode, before the app is installed. iOS shows the text in the
   permission pop-ups and closes the app if a key is missing.
6. On **Signing & Capabilities**, choose your **Team** (sign in with your Apple ID).
7. Plug in the iPhone, pick it in the run-destination menu at the top of the window
   (it must say your iPhone's name, not **My Mac**), and press **Run**. The
   first time, enable **Settings > Privacy & Security > Developer Mode** on the
   iPhone, and trust your developer certificate under **Settings > General > VPN &
   Device Management**.

### Build problems

- **Cannot find 'UIApplication' in scope**, or other iOS-only errors: the run
  destination is **My Mac**. Pick your iPhone (or an iPhone simulator) at the top of
  the window. The simulator builds the app but has no Bluetooth, so test on the phone.
- **Invalid redeclaration of 'ContentView'** or **'main' attribute can only apply to
  one type**: the generated files from step 2 are still in the project. Delete the
  old ones and rename any `… 2.swift` file to drop the ` 2`.
- **The app expires after 7 days**: plug the phone in and press **Run** again. This
  re-signs it for another 7 days and keeps the app's data.

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
