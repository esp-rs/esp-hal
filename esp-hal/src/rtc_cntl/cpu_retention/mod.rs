//! Retention of the CPU power domain across a light sleep.

// The chips whose frames this crate describes. A software-retention chip that is not in the list
// keeps the CPU domain powered, because it has no layout to save into. The list grows with each
// chip.
#[cfg(all(cpu_retention = "software", esp32c6))]
mod chips;
#[cfg(all(cpu_retention = "software", esp32c6))]
mod device_regs;
#[cfg(all(cpu_retention = "software", esp32c6))]
mod frames;
#[cfg(all(cpu_retention = "software", esp32c6))]
mod software;
#[cfg(all(cpu_retention = "software", esp32c6))]
pub(crate) use software::{disarm_wake_stub, sleep_retained};

#[cfg(cpu_retention = "rtc_cntl")]
mod rtc_cntl;
#[cfg(cpu_retention = "rtc_cntl")]
pub(crate) use rtc_cntl::*;

#[cfg(any(cpu_retention = "rtc_cntl", esp32c6))]
pub(crate) mod memory;
#[cfg(any(cpu_retention = "rtc_cntl", esp32c6))]
pub(crate) use memory::installed_buffer_ptr;

/// A software-retention chip with no frame layout yet cannot retain the CPU.
#[cfg(all(cpu_retention = "software", not(esp32c6)))]
pub(crate) fn installed_buffer_ptr() -> Option<*mut u8> {
    None
}
