# esp-rustls-provider

## NO support for targets w/o atomics

While most dependencies can be used with `portable-atomic` that's unfortunately not true for Rustls itself. It needs `alloc::sync::Arc` in a lot of places.

This means that ESP32-S2, ESP32-C2 and ESP32-C3 are NOT supported.

## Status

This crate is currently experimental/preview. It's not available on crates.io

Currently this is basically a copy of [Rustls' provider example](https://github.com/rustls/rustls/tree/main/provider-example) (minus std support, plus some helpers).

When [rustls-rustcrypto](https://crates.io/crates/rustls-rustcrypto) becomes fully no-std, this should depend on that instead.

There are still some more things to do
- Add async wrappers
- Implement optional traits from embedded-io

In future we probably want to add hardware acceleration
