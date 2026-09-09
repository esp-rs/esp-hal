//! Retention buffer layout and device regions, per chip.

#[cfg(esp32c6)]
pub(crate) mod esp32c6;

#[cfg(esp32c6)]
use esp32c6 as chip;

// The metadata carries the size, because the install API checks the buffer against it before any
// frame code runs.
const _: () = ::core::assert!(chip::BUFFER_SIZE == super::memory::buffer_size());
