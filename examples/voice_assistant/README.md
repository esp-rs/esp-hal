# Voice Assistant

A voice assistant for the
[ESP32-S31-Korvo-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-korvo-1/index.html)
with its 800x480 LCD subboard.

## Running

```shell
cargo xtask run voice_assistant esp32s31
```

Without an API key, the assistant only recognizes the light commands and does not use Wi-Fi. To
also answer questions with Claude or OpenAI, set `SSID`, `PASSWORD`, and one of `ANTHROPIC_API_KEY`
or `OPENAI_API_KEY` before you build:

```shell
SSID=... PASSWORD=... ANTHROPIC_API_KEY=... cargo xtask run voice_assistant esp32s31
```

The API key is stored in the firmware in plain text, and anyone with access to the board can read
it. Use a key with a spending limit.

## Use

- Tap the talk button, say a phrase, and tap the talk button again.
- The phrases button shows the phrases the assistant accepts, for example "turn the light blue" or
  "tell me a joke".
- Light commands set the color of the board's RGB LED. Claude or OpenAI answers the other phrases.
- The screen shows the recognized phrase and the answer.

## How it works

- I2S records the two microphones through the ES8389 codec. An interrupt executor runs the audio
  task, so recording continues while other tasks block, for example during a TLS handshake.
- The second core recognizes the phrase on the device with MultiNet7 from Espressif's ESP-SR,
  through the [`esp-sr`](https://github.com/SergioGasquez/esp-sr) crate. MultiNet recognizes a
  fixed list of phrases. It does not transcribe arbitrary speech.
- The LCD_CAM sends the frame buffer in PSRAM to the panel in DPI mode. Speech recognition takes so
  much PSRAM bandwidth that the DMA cannot read the frame buffer directly. So the DMA sends two
  bounce buffers in internal RAM, and an interrupt refills each one while the DMA sends the other.
- Questions go to the Claude or OpenAI API over HTTPS. `embedded-tls` verifies the server
  certificates.
- The CPU reads the Goodix touch controller over I2C.

ESP-SR's prebuilt libraries and model are under Espressif's license, see the `esp-sr` repository.
