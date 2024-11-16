//! # The `esp-hal` Prelude
//!
//! ## Overview
//! The prelude is the list of things that `esp-hal` automatically imports into
//! every program. It’s kept as small as possible, and is focused on
//! things, particularly traits, which are used in almost every single Rust
//! program.

pub use imp::*;

#[doc(hidden)]
mod imp {
    #[doc(hidden)]
    pub use fugit::{ExtU64 as _, RateExtU32 as _};
    #[doc(hidden)]
    pub use nb;

    #[cfg(dac)]
    pub use crate::analog::dac::Instance as _esp_hal_analog_dac_Instance;
    #[cfg(any(dport, pcr, system))]
    pub use crate::clock::Clock as _esp_hal_clock_Clock;
    #[cfg(gpio)]
    pub use crate::gpio::{
        InputPin as _esp_hal_gpio_InputPin,
        OutputPin as _esp_hal_gpio_OutputPin,
        Pin as _esp_hal_gpio_Pin,
    };
    #[cfg(any(i2c0, i2c1))]
    pub use crate::i2c::master::Instance as _esp_hal_i2c_master_Instance;
    #[cfg(ledc)]
    pub use crate::ledc::{
        channel::{
            ChannelHW as _esp_hal_ledc_channel_ChannelHW,
            ChannelIFace as _esp_hal_ledc_channel_ChannelIFace,
        },
        timer::{
            TimerHW as _esp_hal_ledc_timer_TimerHW,
            TimerIFace as _esp_hal_ledc_timer_TimerIFace,
        },
    };
    #[cfg(any(timg0, timg1))]
    pub use crate::timer::timg::{
        Instance as _esp_hal_timer_timg_Instance,
        TimerGroupInstance as _esp_hal_timer_timg_TimerGroupInstance,
    };
    #[cfg(any(systimer, timg0, timg1))]
    pub use crate::timer::Timer as _esp_hal_timer_Timer;
    #[cfg(any(uart0, uart1, uart2))]
    pub use crate::uart::Instance as _esp_hal_uart_Instance;
    pub use crate::{clock::CpuClock, entry, macros::*, InterruptConfigurable};
}
