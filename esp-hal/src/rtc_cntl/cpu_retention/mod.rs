//! Retention of the CPU power domain across a light sleep.

cfg_select! {
    cpu_retention = "software" => {
        mod software;
        pub(crate) use software::{
            disarm_wake_stub,
            enter_sleep_with_retention,
            finish_cpu_retention,
            sleep_retained,
        };
        #[cfg(feature = "rt")]
        pub(crate) use software::configure_cpu_retention;
        #[cfg(all(multi_core, feature = "rt"))]
        pub(crate) use software::rendezvous;
    }

    cpu_retention = "rtc_cntl" => {
        mod rtc_cntl;
        pub(crate) use rtc_cntl::*;
    }

    _ => {}
}

pub(crate) mod memory;
pub(crate) use memory::installed_buffer_ptr;
