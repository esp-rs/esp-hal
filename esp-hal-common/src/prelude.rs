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
pub use embedded_hal::{
    digital::v2::{
        InputPin as _embedded_hal_digital_v2_InputPin,
        OutputPin as _embedded_hal_digital_v2_OutputPin,
        StatefulOutputPin as _embedded_hal_digital_v2_StatefulOutputPin,
        ToggleableOutputPin as _embedded_hal_digital_v2_ToggleableOutputPin,
    },
    prelude::*,
};
#[cfg(feature = "async")]
pub use embedded_hal_async::{
    delay::DelayUs as _embedded_hal_async_delay_DelayUs,
    digital::Wait as _embedded_hal_async_digital_Wait,
    i2c::I2c as _embedded_hal_async_i2c_I2c,
    spi::SpiBus as _embedded_hal_async_spi_SpiBus,
    spi::SpiDevice as _embedded_hal_async_spi_SpiDevice,
};
pub use fugit::{
    ExtU32 as _fugit_ExtU32,
    ExtU64 as _fugit_ExtU64,
    RateExtU32 as _fugit_RateExtU32,
    RateExtU64 as _fugit_RateExtU64,
};
pub use nb;

#[cfg(any(apb_saradc, sens))]
pub use crate::analog::AnalogExt as _esp_hal_analog_AnalogExt;
#[cfg(any(gdma, pdma))]
pub use crate::dma::{
    DmaTransfer as _esp_hal_dma_DmaTransfer,
    DmaTransferRxTx as _esp_hal_dma_DmaTransferRxTx,
};
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
#[cfg(any(dport, pcr, system))]
pub use crate::system::SystemExt as _esp_hal_system_SystemExt;
#[cfg(any(timg0, timg1))]
pub use crate::timer::{
    Instance as _esp_hal_timer_Instance,
    TimerGroupInstance as _esp_hal_timer_TimerGroupInstance,
};
#[cfg(any(uart0, uart1, uart2))]
pub use crate::uart::{Instance as _esp_hal_uart_Instance, UartPins as _esp_hal_uart_UartPins};
pub use crate::{clock::Clock as _esp_hal_clock_Clock, entry, macros::*};

/// All traits required for using the 1.0.0-alpha.x release of embedded-hal
#[cfg(feature = "eh1")]
pub mod eh1 {
    #[cfg(any(twai0, twai1))]
    pub use embedded_can::{
        blocking::Can as _embedded_can_blocking_Can,
        nb::Can as _embedded_can_nb_Can,
        Error as _embedded_can_Error,
        Frame as _embedded_can_Frame,
    };
    pub use embedded_hal_1::{
        delay::DelayUs as _embedded_hal_1_delay_DelayUs,
        digital::{
            InputPin as _embedded_hal_1_digital_InputPin,
            OutputPin as _embedded_hal_1_digital_OutputPin,
            StatefulOutputPin as _embedded_hal_1_digital_StatefulOutputPin,
            ToggleableOutputPin as _embedded_hal_1_digital_ToggleableOutputPin,
        },
        i2c::I2c as _embedded_hal_1_i2c_I2c,
        spi::{SpiBus as _embedded_hal_1_spi_SpiBus, SpiDevice as _embedded_hal_1_spi_SpiDevice},
    };
    pub use embedded_hal_nb::{
        serial::{Read as _embedded_hal_nb_serial_Read, Write as _embedded_hal_nb_serial_Write},
        spi::FullDuplex as _embedded_hal_nb_spi_FullDuplex,
    };

    pub use super::*;
}
