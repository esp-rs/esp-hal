//! Miscellaneous simple tests
//!
//! Clock Monitor Test
//!
//! Ensure invariants of locks are upheld.
//!
//! Async Delay Test
//!     Specifically tests the various implementations of the
//!     `embedded_hal_async::delay::DelayNs` trait.
//!     This test does not configure embassy, as it doesn't need a timer queue
//!     implementation or an embassy time driver.
//!
//! DMA macro tests
//!
//! DMA Mem2Mem Tests
//!
//! Initialization tests
//!
//! The goal of this test suite is to collect smaller, simpler test cases, to keep the overall
//! number of test suites low(er).

//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

#[path = "misc_non_drivers/clock_monitor.rs"]
mod clock_monitor;

#[path = "misc_non_drivers/critical_section.rs"]
mod critical_section;

#[path = "misc_non_drivers/delay_async.rs"]
#[cfg(timergroup_driver_supported)]
mod delay_async;

#[path = "misc_non_drivers/dma_macros.rs"]
#[cfg(dma_driver_supported)]
mod dma_macros;

#[path = "misc_non_drivers/dma_mem2mem.rs"]
#[cfg(not(esp32))] // TODO: dma_supports_mem2mem
#[cfg(dma_driver_supported)]
mod dma_mem2mem;

#[path = "misc_non_drivers/init.rs"]
mod init;

#[path = "misc_non_drivers/simple.rs"]
mod simple;

#[path = "misc_non_drivers/interrupt_nesting.rs"]
mod interrupt_nesting;
