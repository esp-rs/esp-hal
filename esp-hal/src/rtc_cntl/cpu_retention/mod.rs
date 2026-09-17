//! Retention of the CPU power domain across a light sleep.

cfg_select! {
    cpu_retention = "software" => {
        mod software;
        #[cfg(feature = "rt")]
        pub(crate) use software::configure_cpu_retention;
        #[cfg(all(multi_core, feature = "rt"))]
        pub(crate) use software::rendezvous;
        pub(crate) use software::{
            enter_sleep_with_retention,
            finish_cpu_retention,
            sleep_retained,
        };
    }

    cpu_retention = "rtc_cntl" => {
        mod rtc_cntl;
        pub(crate) use rtc_cntl::*;
    }

    _ => {}
}

pub(crate) mod memory;
pub(crate) use memory::installed_buffer_ptr;
