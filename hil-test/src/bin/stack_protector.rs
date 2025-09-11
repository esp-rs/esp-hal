//! Tests that the `-Zstack-protector=all` works as expected.
//!
//! The stack protector is enabled by setting HIL_ENABLE_STACK_PROTECTOR. The
//! xtask recognizes this and sets the cargo config to `.cargo/config_spp.toml`,
//! which enables the feature.

//% CARGO-CONFIG: target.'cfg(target_arch = "riscv32")'.rustflags = [ "-Z", "stack-protector=all" ]
//% CARGO-CONFIG: target.'cfg(target_arch = "xtensa")'.rustflags = [ "-Z", "stack-protector=all" ]
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-alloc

#![no_std]
#![no_main]

use hil_test as _;

#[inline(never)]
fn trigger_overflow() {
    // Aim for the middle of heap: these are roughly DRAM_LEN - 16k
    const SIZE: usize = if cfg!(esp32) {
        160 * 1024
    } else if cfg!(esp32c2) {
        176 * 1024
    } else if cfg!(esp32c3) {
        297 * 1024
    } else if cfg!(esp32c6) {
        425 * 1024
    } else if cfg!(esp32h2) {
        235 * 1024
    } else if cfg!(esp32s2) {
        169 * 1024
    } else if cfg!(esp32s3) {
        322 * 1024
    } else {
        unreachable!()
    };
    let mut stack = core::hint::black_box([0u8; SIZE]);
    stack[SIZE - 1] = 42;
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());
        // Have some data that we can overflow into.
        esp_alloc::heap_allocator!(size: 32 * 1024);
    }

    #[test]
    fn should_be_ok() {
        assert_eq!(1 + 1, 2);
    }

    #[test]
    #[should_panic]
    fn should_trigger_panic() {
        trigger_overflow();
    }
}
