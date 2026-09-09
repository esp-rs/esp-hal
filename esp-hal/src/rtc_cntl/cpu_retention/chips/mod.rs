//! Retention buffer layout and device regions, per chip.

#[cfg_attr(any(esp32c6, esp32h2), path = "c6_h2.rs")]
#[cfg_attr(esp32c5, path = "c5_c61.rs")]
pub(crate) mod chip;

// The metadata carries the size, because the install API checks the buffer against it before any
// frame code runs.
const _: () = ::core::assert!(chip::BUFFER_SIZE == super::memory::buffer_size());
