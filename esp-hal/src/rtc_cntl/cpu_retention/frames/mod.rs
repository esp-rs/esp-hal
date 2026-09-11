//! Sleep frame layouts.
//!
//! The chips fall into three groups, and the groups do not share a layout. See
//! `components/esp_hw_support/lowpower/port/<chip>/rvsleep-frames.h`.

/// Declares the non-critical frame and the matched pair that moves it.
///
/// One list drives the struct, the save and the restore, so the three cannot drift apart.
macro_rules! non_critical_frame {
    ($($field:ident = $csr:expr),* $(,)?) => {
        /// Registers that Rust saves, because the CPU is usable while they move.
        #[repr(C)]
        pub(crate) struct NonCriticalSleepFrame {
            $($field: u32,)*
        }

        impl NonCriticalSleepFrame {
            /// Reads every CSR of the frame.
            ///
            /// # Safety
            ///
            /// The caller must own the CPU state that the frame describes.
            pub(crate) unsafe fn save(&mut self) {
                $(self.$field = unsafe { read_csr!($csr) };)*
            }

            /// Writes every CSR of the frame back.
            ///
            /// # Safety
            ///
            /// The frame must hold a state that [`Self::save`] read from this core.
            pub(crate) unsafe fn restore(&self) {
                $(unsafe { write_csr!($csr, self.$field) };)*
            }
        }
    };
}

pub(crate) use non_critical_frame;

#[cfg_attr(cpu_retention_frame = "c6_h2", path = "c6_h2.rs")]
#[cfg_attr(cpu_retention_frame = "clic", path = "clic.rs")]
#[cfg_attr(cpu_retention_frame = "p4", path = "p4.rs")]
#[cfg_attr(cpu_retention_frame = "s31", path = "s31.rs")]
pub(crate) mod chip;
