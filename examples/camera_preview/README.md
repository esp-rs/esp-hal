# Camera Preview

A live camera preview for the
[ESP32-P4-Function-EV-Board](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html)
with its SC2336 camera module and 1024x600 EK79007 MIPI-DSI LCD.

## Running

Connect the camera to the MIPI-CSI connector and the LCD to the MIPI-DSI connector, then run:

```shell
cargo xtask run camera_preview esp32p4
```

The LCD shows the image from the camera. Every 5 seconds, the log shows the camera frame rate,
the shown frame rate, and the time of the colour conversion.

## How it works

- The SC2336 camera sends 1024x600 RAW8 Bayer frames at 30 frames per second over two MIPI-CSI
  lanes. A `CsiStream` writes them into two frame buffers in PSRAM.
- The first frame sets the white balance: the example scales red and blue so that their mean
  matches green.
- The CPU converts each new frame into the RGB565 frame buffer of the LCD, which has the same
  size. `DsiDpi::commit` then shows it, and `DsiDpi::wait_for_commit` waits until the LCD shows
  it.
- The MIPI-DSI host sends the frame buffer to the LCD in video mode.
