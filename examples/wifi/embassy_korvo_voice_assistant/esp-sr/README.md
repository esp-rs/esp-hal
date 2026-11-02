# esp-sr

Offline speech command recognition on the ESP32-S31 with MultiNet7 from Espressif's
[ESP-SR](https://github.com/espressif/esp-sr), for esp-hal.

MultiNet recognizes which one of a list of English phrases was said, such as "turn the light red",
entirely on the chip. It does not transcribe arbitrary speech.

```rust,ignore
let mut multinet = esp_sr::MultiNet::new([
    esp_sr::Phrase::new(c"turn the light red"),
    esp_sr::Phrase::new(c"turn off the light"),
])?;
match multinet.recognize(&speech) {
    esp_sr::Recognition::Phrase { index, .. } => { /* ... */ }
    esp_sr::Recognition::Nothing { heard } => { /* ... */ }
}
```

See the crate documentation for the requirements: PSRAM, and a core of its own for MultiNet.

## What is in here

- `lib/esp32s31`: Espressif's prebuilt ESP-SR 2.6.0 libraries that MultiNet needs, prepared by
  `tools/vendor.py`: Espressif's vendor relocations, which only GNU ld accepts, are neutralized
  (the assembler already encodes what they describe), and the C library imports are renamed with
  an `esp_sr_` prefix.
- `model`: the English MultiNet7 model, which the crate embeds in the firmware.
- `src`: Rust for everything the libraries import: the C library functions, `printf` through small
  assembly shims, ESP-SR's phrase list, cJSON's parser and ESP-DL's real FFT.

Building needs only Rust: no C compiler, no GNU ld, no ESP-IDF. Updating the blobs needs
Espressif's RISC-V binutils and an ESP-SR release, see `tools/vendor.py`.

## License

The Rust code is licensed under MIT or Apache-2.0. The libraries in `lib` and the model in `model`
are Espressif's, under the license in `LICENSE-ESP-SR`, which grants their use on Espressif chips.
