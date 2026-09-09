//! Retention of the CPU power domain across a light sleep.

#[cfg(all(cpu_retention = "software", supports_cpu_power_down))]
mod chips;
#[cfg(all(cpu_retention = "software", supports_cpu_power_down))]
mod device_regs;
#[cfg(all(cpu_retention = "software", supports_cpu_power_down))]
mod frames;
#[cfg(all(cpu_retention = "software", supports_cpu_power_down))]
mod software;
#[cfg(all(cpu_retention = "software", supports_cpu_power_down))]
pub(crate) use software::{disarm_wake_stub, sleep_retained};

#[cfg(cpu_retention = "rtc_cntl")]
mod rtc_cntl;
#[cfg(cpu_retention = "rtc_cntl")]
pub(crate) use rtc_cntl::*;

#[cfg(supports_cpu_power_down)]
pub(crate) mod memory;
#[cfg(supports_cpu_power_down)]
pub(crate) use memory::installed_buffer_ptr;

/// A software-retention chip with no frame layout yet cannot retain the CPU.
#[cfg(all(cpu_retention = "software", not(supports_cpu_power_down)))]
pub(crate) fn installed_buffer_ptr() -> Option<*mut u8> {
    None
}
