// Fallback target for `mod implementation;` in `mod.rs`.
//
// Shared controllers set `adc.controller` to `rtc` or `digital`. ESP32, ESP32-P4, and ESP32-S31
// use chip path attrs. This file is only compiled when none of those match.
compile_error!(
    "This chip has ADC support enabled in `esp-metadata` but no ADC implementation was selected. \
     Set `adc.controller` to \"rtc\" or \"digital\", or add a chip path attr in `mod.rs`."
);
