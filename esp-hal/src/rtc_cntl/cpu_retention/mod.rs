//! Retention of the CPU power domain across a light sleep.

cfg_select! {
    all(cpu_retention = "software", supports_cpu_power_down) => {
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
            feature = "rt",
            feature = "unstable"
        ))]
        pub(crate) mod rendezvous;

        /// The answers of a chip that sleeps as one core.
        #[cfg(not(all(multi_core, feature = "rt", feature = "unstable")))]
        pub(crate) mod rendezvous {
            /// One core has nothing to rendezvous with, so its caller always sleeps.
            #[crate::ram]
            pub(crate) fn engage() -> bool {
                true
            }

            /// No core saves itself for another one, so the sleep path stalls the other core as before.
            #[cfg(multi_core)]
            #[crate::ram]
            pub(crate) fn helper_enlisted() -> bool {
                false
            }

            /// Without a rendezvous, a second running core cannot save itself, so the CPU domain must
            /// keep its power.
            #[cfg(multi_core)]
            #[crate::ram]
            pub(crate) fn retention_allowed() -> bool {
                !crate::soc::cpu_control::is_running(crate::system::Cpu::AppCpu)
            }

            #[crate::ram]
            pub(crate) fn finish() {}
        }
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
#[cfg(all(cpu_retention = "software", not(supports_cpu_power_down)))]
pub(crate) fn installed_buffer_ptr() -> Option<core::ptr::NonNull<u8>> {
    None
}
