# Augment-Glasses
See-through captioning glasses using virtual-image optics to display real-time speech-to-text, helping deaf and hard of hearing users follow conversations

## Arduino sketch

Open [speech_text_screen.ino](Arduino/speech_text_screen/speech_text_screen.ino)
in the Arduino IDE and select **ESP32S3 Dev Module**. The sketch is in a matching
folder as required by the Arduino IDE.

Wiring (INMP441 I2S microphone):

| INMP441 | ESP32-S3 |
|---------|----------|
| SCK     | GPIO 4   |
| WS      | GPIO 5   |
| SD      | GPIO 6   |
| L/R     | GND      |
| VDD     | 3V3      |

The sketch captures 16 kHz, 16-bit mono PCM in 20 ms frames and queues them for
sending. Once a second, the Serial Monitor (115200 baud) shows the RMS and peak level
in dBFS, frames per second (should be 50), clipped samples, and dropped frames. Adjust
`MIC_GAIN_DB` until speech peaks reach about -12 to -6 dBFS without clipping.

The ESP32 does not use WiFi. The planned audio path uses only BLE: the ESP32 sends
audio to the phone, the phone runs speech-to-text, and the phone sends the text back
to the ESP32 for display. The BLE link and display output are not implemented yet.
