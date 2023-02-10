//! The prelude
//!
//! Re-exports all traits required for interacting with the various peripheral
//! drivers implemented in this crate.

pub use embedded_hal::{
    digital::v2::{
        InputPin as _embedded_hal_digital_v2_InputPin,
        OutputPin as _embedded_hal_digital_v2_OutputPin,
        StatefulOutputPin as _embedded_hal_digital_v2_StatefulOutputPin,
        ToggleableOutputPin as _embedded_hal_digital_v2_ToggleableOutputPin,
    },
    prelude::*,
};
pub use fugit::{
    ExtU32 as _fugit_ExtU32,
    ExtU64 as _fugit_ExtU64,
    RateExtU32 as _fugit_RateExtU32,
    RateExtU64 as _fugit_RateExtU64,
};
pub use nb;

#[cfg(any(esp32c2, esp32c3))]
pub use crate::analog::SarAdcExt as _esp_hal_analog_SarAdcExt;
#[cfg(any(esp32, esp32s2, esp32s3))]
pub use crate::analog::SensExt as _esp_hal_analog_SensExt;
#[cfg(rmt)]
pub use crate::pulse_control::{
    ConfiguredChannel as _esp_hal_pulse_control_ConfiguredChannel,
    OutputChannel as _esp_hal_pulse_control_OutputChannel,
};
#[cfg(any(esp32, esp32s2))]
pub use crate::spi::dma::WithDmaSpi3 as _esp_hal_spi_dma_WithDmaSpi3;
pub use crate::{
    clock::Clock as _esp_hal_clock_Clock,
    dma::{
        DmaTransfer as _esp_hal_dma_DmaTransfer,
        DmaTransferRxTx as _esp_hal_dma_DmaTransferRxTx,
    },
    entry,
    gpio::{
        InputPin as _esp_hal_gpio_InputPin,
        OutputPin as _esp_hal_gpio_OutputPin,
        Pin as _esp_hal_gpio_Pin,
    },
    i2c::Instance as _esp_hal_i2c_Instance,
    ledc::{
        channel::{
            ChannelHW as _esp_hal_ledc_channel_ChannelHW,
            ChannelIFace as _esp_hal_ledc_channel_ChannelIFace,
        },
        timer::{
            TimerHW as _esp_hal_ledc_timer_TimerHW,
            TimerIFace as _esp_hal_ledc_timer_TimerIFace,
        },
    },
    macros::*,
    spi::{
        dma::WithDmaSpi2 as _esp_hal_spi_dma_WithDmaSpi2,
        Instance as _esp_hal_spi_Instance,
        InstanceDma as _esp_hal_spi_InstanceDma,
    },
    system::SystemExt as _esp_hal_system_SystemExt,
    timer::{
        Instance as _esp_hal_timer_Instance,
        TimerGroupInstance as _esp_hal_timer_TimerGroupInstance,
    },
    uart::{Instance as _esp_hal_uart_Instance, UartPins as _esp_hal_uart_UartPins},
};

/// All traits required for using the 1.0.0-alpha.x release of embedded-hal
#[cfg(feature = "eh1")]
pub mod eh1 {
    pub use embedded_hal_1::{
        delay::DelayUs as _embedded_hal_delay_blocking_DelayUs,
        digital::{
            InputPin as _embedded_hal_digital_blocking_InputPin,
            OutputPin as _embedded_hal_digital_blocking_OutputPin,
            StatefulOutputPin as _embedded_hal_digital_blocking_StatefulOutputPin,
            ToggleableOutputPin as _embedded_hal_digital_blocking_ToggleableOutputPin,
        },
        i2c::I2c as _embedded_hal_i2c_blocking_I2c,
        spi::{
            SpiBus as _embedded_hal_spi_blocking_SpiBus,
            SpiBusFlush as _embedded_hal_spi_blocking_SpiBusFlush,
            SpiBusRead as _embedded_hal_spi_blocking_SpiBusRead,
            SpiBusWrite as _embedded_hal_spi_blocking_SpiBusWrite,
        },
    };
    pub use embedded_hal_nb::{
        serial::{Read as _embedded_hal_nb_serial_Read, Write as _embedded_hal_nb_serial_Write},
        spi::FullDuplex as _embedded_hal_nb_spi_FullDuplex,
    };
    pub use fugit::{
        ExtU32 as _fugit_ExtU32,
        ExtU64 as _fugit_ExtU64,
        RateExtU32 as _fugit_RateExtU32,
        RateExtU64 as _fugit_RateExtU64,
    };
    pub use nb;

    #[cfg(any(esp32c2, esp32c3))]
    pub use crate::analog::SarAdcExt as _esp_hal_analog_SarAdcExt;
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub use crate::analog::SensExt as _esp_hal_analog_SensExt;
    #[cfg(rmt)]
    pub use crate::pulse_control::{
        ConfiguredChannel as _esp_hal_pulse_control_ConfiguredChannel,
        OutputChannel as _esp_hal_pulse_control_OutputChannel,
    };
    #[cfg(any(esp32, esp32s2))]
    pub use crate::spi::dma::WithDmaSpi3 as _esp_hal_spi_dma_WithDmaSpi3;
    pub use crate::{
        clock::Clock as _esp_hal_clock_Clock,
        dma::{
            DmaTransfer as _esp_hal_dma_DmaTransfer,
            DmaTransferRxTx as _esp_hal_dma_DmaTransferRxTx,
        },
        gpio::{
            InputPin as _esp_hal_gpio_InputPin,
            OutputPin as _esp_hal_gpio_OutputPin,
            Pin as _esp_hal_gpio_Pin,
        },
        i2c::Instance as _esp_hal_i2c_Instance,
        ledc::{
            channel::{
                ChannelHW as _esp_hal_ledc_channel_ChannelHW,
                ChannelIFace as _esp_hal_ledc_channel_ChannelIFace,
            },
            timer::{
                TimerHW as _esp_hal_ledc_timer_TimerHW,
                TimerIFace as _esp_hal_ledc_timer_TimerIFace,
            },
        },
        macros::*,
        spi::{
            dma::WithDmaSpi2 as _esp_hal_spi_dma_WithDmaSpi2,
            Instance as _esp_hal_spi_Instance,
            InstanceDma as _esp_hal_spi_InstanceDma,
        },
        system::SystemExt as _esp_hal_system_SystemExt,
        timer::{
            Instance as _esp_hal_timer_Instance,
            TimerGroupInstance as _esp_hal_timer_TimerGroupInstance,
        },
        uart::{Instance as _esp_hal_serial_Instance, UartPins as _esp_hal_serial_UartPins},
    };
}
