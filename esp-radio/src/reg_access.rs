//! Minimal chip-agnostic access to PAC register blocks.

/// Chip-selected PAC crate re-export.
#[cfg(esp32)]
pub(crate) use esp32 as pac;
#[cfg(esp32c2)]
pub(crate) use esp32c2 as pac;
#[cfg(esp32c3)]
pub(crate) use esp32c3 as pac;
#[cfg(esp32c5)]
pub(crate) use esp32c5 as pac;
#[cfg(esp32c6)]
pub(crate) use esp32c6 as pac;
#[cfg(esp32h2)]
pub(crate) use esp32h2 as pac;
#[cfg(esp32s2)]
pub(crate) use esp32s2 as pac;
#[cfg(esp32s3)]
pub(crate) use esp32s3 as pac;

/// Get a peripheral register block reference from the PAC. 
#[macro_export]
macro_rules! periph {
    ($P:ident) => {{
        unsafe { &*$crate::reg_access::pac::$P::ptr() }
    }};
}