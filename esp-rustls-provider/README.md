# esp-rustls-provider

## NO support for targets w/o atomics

While most dependencies can be used with `portable-atomic` that's unfortunately not true for Rustls itself. It needs `alloc::sync::Arc` in a lot of places.

This means that ESP32-S2, ESP32-C2 and ESP32-C3 are NOT supported.

## Status

This crate is currently experimental/preview. It's not available on crates.io and might be limited in functionality.
