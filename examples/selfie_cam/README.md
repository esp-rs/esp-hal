# Selfie Camera

A selfie camera for the
[ESP32-S31-Korvo-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-korvo-1/index.html)
with its 800x480 LCD subboard and camera module.

## Running

Insert a FAT-formatted microSD card, then run:

```shell
cargo xtask run selfie_cam esp32s31
```

Without a card, the camera still runs, but it cannot save photos. To keep about ten photos in
PSRAM instead, until a reset, set `STORE_IN_RAM` in `src/main.rs`.

The runner flashes the image with an 80 MHz flash clock. With the 40 MHz default, the panel
shows stripes while a photo loads.

## Use

- The screen shows the image from the camera.
- The SET key (or BOOT) takes a 1280x720 photo and saves it as `DCIM/IMG_NNNN.JPG` on the card.
- GALLERY shows the saved photos. `<` and `>` go through them, DELETE (tapped twice) deletes
  one, and CAMERA (or SET) goes back to the camera.

## How it works

- The SC101IOT camera sends 1280x720 YUV422 at about 15 frames per second. The LCD_CAM camera
  unit converts it to RGB565, and its DMA writes each frame into a frame buffer in PSRAM. An
  interrupt at the end of each frame moves the DMA to the next free frame buffer.
- The LCD_CAM sends the picture to the panel in DPI mode. PSRAM is too busy with the camera to
  feed the panel, so the LCD DMA reads two small bounce buffers in internal RAM. An interrupt on
  the second core draws the next rows into each finished buffer: the scaled camera picture and
  the buttons on top of it.
- A photo is a JPEG file (`jpeg-encoder`) on the FAT volume (`embedded-fatfs`) of the microSD
  card, which runs on the SDMMC host with a 4-bit bus. The gallery decodes it with `zune-jpeg`.
