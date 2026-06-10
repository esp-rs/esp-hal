cfg_if::cfg_if! {
    if #[cfg(feature="interrupts")]
    {
        // RISCV ULP specific interrupt handlers, critical section
        #[cfg(any(esp32s2, esp32s3))]
        pub mod ulp_core;
        #[cfg(any(esp32s2, esp32s3))]
        pub use ulp_core::*;

        #[cfg(esp32c6)]
        pub mod lp_core;
        #[cfg(esp32c6)]
        pub use lp_core::*;

        /// Portable interrupt binding and handling code
        pub mod shared;
        pub use shared::*;
    } else {
        // If interrupt handling is not enabled, need to provide a stub DefaultHandler,
        // to make the linker happy.
        #[doc(hidden)]
        #[unsafe(export_name = "DefaultHandler")]
        #[unsafe(link_section = ".trap.rust")]
        pub fn stub_default_handler() {}
    }
}

/// LP core critical section implementation.
/// Works with or without the interrupts feature.
mod critical_section;
