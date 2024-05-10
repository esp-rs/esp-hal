//! The prelude
//!
//! Re-exports all traits required for interacting with the various peripheral
//! drivers implemented in this crate.

pub use embedded_dma::{
    ReadBuffer as _embedded_dma_ReadBuffer,
    ReadTarget as _embedded_dma_ReadTarget,
    Word as _embedded_dma_Word,
    WriteBuffer as _embedded_dma_WriteBuffer,
    WriteTarget as _embedded_dma_WriteTarget,
};
pub use fugit::{ExtU64 as _fugit_ExtU64, RateExtU32 as _fugit_RateExtU32};
pub use nb;

#[cfg(any(dport, pcr, system))]
pub use crate::clock::Clock as _esp_hal_clock_Clock;
#[cfg(gpio)]
pub use crate::gpio::{
    InputPin as _esp_hal_gpio_InputPin,
    OutputPin as _esp_hal_gpio_OutputPin,
    Pin as _esp_hal_gpio_Pin,
};
#[cfg(any(i2c0, i2c1))]
pub use crate::i2c::Instance as _esp_hal_i2c_Instance;
#[cfg(ledc)]
pub use crate::ledc::{
    channel::{
        ChannelHW as _esp_hal_ledc_channel_ChannelHW,
        ChannelIFace as _esp_hal_ledc_channel_ChannelIFace,
    },
    timer::{TimerHW as _esp_hal_ledc_timer_TimerHW, TimerIFace as _esp_hal_ledc_timer_TimerIFace},
};
#[cfg(any(timg0, timg1))]
pub use crate::timer::timg::{
    Instance as _esp_hal_timer_timg_Instance,
    TimerGroupInstance as _esp_hal_timer_timg_TimerGroupInstance,
};
#[cfg(any(uart0, uart1, uart2))]
pub use crate::uart::{Instance as _esp_hal_uart_Instance, UartPins as _esp_hal_uart_UartPins};
pub use crate::{entry, macros::*};
