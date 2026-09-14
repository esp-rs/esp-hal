//! Retention of the CPU power domain across a light sleep.

#[cfg_attr(esp32c3, path = "chips/esp32c3.rs")]
#[cfg_attr(esp32s3, path = "chips/esp32s3.rs")]
#[cfg_attr(cpu_retention = "software", path = "chips/software.rs")]
mod chip;
pub(crate) use chip::*;

cfg_select! {
    cpu_retention = "software" => {
        mod device_regs;
        mod frames;

        mod software;
        pub(crate) use software::{disarm_wake_stub, sleep_retained};

        // The rendezvous needs the IPC path, which `interrupt::ipc` gives to a
        // multi-core chip with the `rt` and the `unstable` features.
        #[cfg(all(
            multi_core,
            feature = "rt"
        ))]
        pub(crate) mod rendezvous;
    }

    cpu_retention = "rtc_cntl" => {
        mod rtc_cntl;
        pub(crate) use rtc_cntl::*;
    }

    _ => {}
}

cfg_select! {
    supports_cpu_power_down => {
        pub(crate) mod memory;
        pub(crate) use memory::installed_buffer_ptr;
    }
    _ => {
        /// A software-retention chip with no frame layout yet cannot retain the CPU.
        pub(crate) fn installed_buffer_ptr() -> Option<core::ptr::NonNull<u8>> {
            None
        }
    }
}
