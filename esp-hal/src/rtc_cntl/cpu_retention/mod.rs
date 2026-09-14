//! Retention of the CPU power domain across a light sleep.

cfg_select! {
    cpu_retention = "software" => {
        mod chips;
        mod device_regs;
        mod frames;

        mod software;
        pub(crate) use software::{disarm_wake_stub, sleep_retained};

        // The rendezvous needs the IPC path, which `interrupt::ipc` gives to a
        // multi-core chip with the `rt` and the `unstable` features. Every other
        // chip sleeps as one core, so the stub below answers for it, and the callers
        // need no condition of their own.
        #[cfg(all(
            multi_core,
            feature = "rt"
        ))]
        pub(crate) mod rendezvous;
    }

    _ => {}
}

cfg_select! {
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
    _ => {}
}

/// A software-retention chip with no frame layout yet cannot retain the CPU.
#[cfg(not(supports_cpu_power_down))]
pub(crate) fn installed_buffer_ptr() -> Option<core::ptr::NonNull<u8>> {
    None
}
