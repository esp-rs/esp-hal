// Fallback target for `mod implementation;` in `mod.rs`.
//
// Every supported chip redirects that module to a real implementation with a `#[cfg_attr(<chip>,
// path = "...")]` arm, so this file is only ever compiled when a chip enables `[device.adc]` in its
// metadata without being listed there. Relying on the default module path keeps the chip list in
// one place; the alternative, a `#[cfg(not(any(...)))]` guard, would have to repeat it.
compile_error!(
    "This chip has ADC support enabled in `esp-metadata` but no ADC implementation module. \
     Add a `#[cfg_attr(<chip>, path = \"...\")]` arm to `esp-hal/src/analog/adc/mod.rs`, or drop \
     `[device.adc]` from the chip's `soc.toml`."
);
