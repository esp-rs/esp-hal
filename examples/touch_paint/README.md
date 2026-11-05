# Touch Paint

A finger-paint program for the
[ESP32-S31-Korvo-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-korvo-1/index.html)
with its 800x480 LCD subboard.

## Running

```shell
cargo xtask run touch_paint esp32s31
```

## Use

- Draw on the white canvas with one finger.
- The toolbar at the top has 8 colors, 3 brush sizes (S, M and L), an eraser and CLEAR.
- The SET button draws a demo picture.

## How it works

- The framebuffer is in PSRAM: 800x480 pixels in RGB565, 768,000 bytes.
- The LCD_CAM sends the framebuffer to the panel in DPI mode. Its DMA reads the framebuffer
  through a circular descriptor chain, so the panel shows each change without a new transfer.
- The CPU reads the Goodix touch controller over I2C every 15 ms and draws lines into the
  framebuffer. After each change, it writes the changed rows back from the data cache to PSRAM,
  where the DMA reads them.

## Extended version

[esp-touch-paint](https://github.com/playfulFence/esp-touch-paint) is an extended version of this
example. It also runs on the ESP32-S3-LCD-EV-Board, and boards share one canvas over ESP-NOW.
