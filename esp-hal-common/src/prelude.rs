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

#[cfg(any(feature = "esp32", feature = "esp32s2"))]
pub use crate::analog::SensExt;
pub use crate::system::SystemExt;

/// All traits required for using the 1.0.0-alpha.x release of embedded-hal
#[cfg(feature = "eh1")]
pub mod eh1 {
    pub use embedded_hal_1::{
        delay::blocking::DelayUs as _embedded_hal_delay_blocking_DelayUs,
        digital::blocking::{
            InputPin as _embedded_hal_digital_blocking_InputPin,
            OutputPin as _embedded_hal_digital_blocking_OutputPin,
            StatefulOutputPin as _embedded_hal_digital_blocking_StatefulOutputPin,
            ToggleableOutputPin as _embedded_hal_digital_blocking_ToggleableOutputPin,
        },
        i2c::blocking::I2c as _embedded_hal_i2c_blocking_I2c,
        serial::nb::{
            Read as _embedded_hal_serial_nb_Read,
            Write as _embedded_hal_serial_nb_Write,
        },
        spi::{
            blocking::{
                SpiBus as _embedded_hal_spi_blocking_SpiBus,
                SpiBusFlush as _embedded_hal_spi_blocking_SpiBusFlush,
                SpiBusRead as _embedded_hal_spi_blocking_SpiBusRead,
                SpiBusWrite as _embedded_hal_spi_blocking_SpiBusWrite,
            },
            nb::FullDuplex as _embedded_hal_spi_nb_FullDuplex,
        },
    };
    pub use fugit::{
        ExtU32 as _fugit_ExtU32,
        ExtU64 as _fugit_ExtU64,
        RateExtU32 as _fugit_RateExtU32,
        RateExtU64 as _fugit_RateExtU64,
    };
    pub use nb;

    pub use crate::system::SystemExt;
}

pub use crate::macros::*;
