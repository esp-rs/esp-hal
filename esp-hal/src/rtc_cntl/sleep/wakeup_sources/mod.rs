cfg_select! {
    sleep_has_wakeup_source_ext0 => {
        mod ext0;
        #[instability::unstable]
        pub use ext0::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_ext1 => {
        mod ext1;
        #[instability::unstable]
        pub use ext1::*;
    }
    _ => {}
}

// TODO: merge with RTCIO?
cfg_select! {
    sleep_has_wakeup_source_gpio => {
        mod gpio;
        #[instability::unstable]
        pub use gpio::*;
    }
    _ => {}
}

// TODO: use a lp_gpio_wakes_from_deep_sleep cfg, or make this universal.
cfg_select! {
    all(sleep_has_wakeup_source_gpio, any(esp32c2, esp32c3, esp32s2, esp32s3)) => {
        mod rtcio;
        #[instability::unstable]
        pub use rtcio::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_timer => {
        mod timer;
        #[instability::unstable]
        pub use timer::*;
    }
    _ => {}
}

// TODO: merge with ULP?
cfg_select! {
    sleep_has_wakeup_source_lp_core => {
        mod lp_core;
        #[instability::unstable]
        pub use lp_core::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_ulp_riscv => {
        mod ulp;
        #[instability::unstable]
        pub use ulp::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_uart => {
        mod uart;
        #[instability::unstable]
        pub use uart::*;
    }
    _ => {}
}
