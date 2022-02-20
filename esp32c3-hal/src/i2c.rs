/// Chip-specific I2C implementation
pub use esp_hal_common::i2c::Pins;
use esp_hal_common::{
    i2c::*,
    pac::{i2c::RegisterBlock, I2C as I2C0},
};

use crate::gpio::{InputSignal, OutputSignal};

i2c_peripheral!(
    OutputSignal,
    InputSignal,
    I2C0: (
        OutputSignal::I2CEXT0_SDA,
        InputSignal::I2CEXT0_SDA,
        OutputSignal::I2CEXT0_SCL,
        InputSignal::I2CEXT0_SCL
    )
);
