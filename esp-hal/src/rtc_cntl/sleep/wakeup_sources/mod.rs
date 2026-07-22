cfg_select! {
    sleep_has_wakeup_source_ext0 => {
        mod ext0;
        pub use ext0::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_ext1 => {
        mod ext1;
        pub use ext1::*;
    }
    _ => {}
}

// TODO: merge with RTCIO?
cfg_select! {
    sleep_has_wakeup_source_gpio => {
        mod gpio;
        pub use gpio::*;
    }
    _ => {}
}

// TODO: use a lp_gpio_wakes_from_deep_sleep cfg, or make this universal.
cfg_select! {
    all(sleep_has_wakeup_source_gpio, any(esp32c2, esp32c3, esp32s2, esp32s3)) => {
        mod rtcio;
        pub use rtcio::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_timer => {
        mod timer;
        pub use timer::*;
    }
    _ => {}
}

// TODO: merge with ULP?
cfg_select! {
    sleep_has_wakeup_source_lp_core => {
        mod lp_core;
        pub use lp_core::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_ulp_riscv => {
        mod ulp;
        pub use ulp::*;
    }
    _ => {}
}

cfg_select! {
    sleep_has_wakeup_source_uart => {
        mod uart;
        pub use uart::*;
    }
    _ => {}
}
