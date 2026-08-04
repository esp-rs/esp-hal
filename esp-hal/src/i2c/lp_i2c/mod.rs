//! Low-power I2C driver

use crate::{
    gpio::{InputPin, LpPin, OutputPin},
    peripherals::LP_I2C0,
};

#[cfg_attr(lp_i2c_master_version = "lp_i2c", path = "lp_i2c.rs")]
#[cfg_attr(lp_i2c_master_version = "rtc_i2c", path = "rtc_i2c.rs")]
mod driver;
pub use driver::*;

/// Trait representing the LP_I2C SDA pin.
pub trait Sda: LpPin + OutputPin + InputPin {
    #[doc(hidden)]
    fn connect_sda(&self);
}

/// Trait representing the LP_I2C SCL pin.
pub trait Scl: LpPin + OutputPin + InputPin {
    #[doc(hidden)]
    fn connect_scl(&self);
}

// Chips with an LP GPIO matrix can route the LP I2C signals to any LP pin.
// Chips without one only expose them on the pads whose LP IO MUX has an LP I2C function. This
// is implemented per-driver.
#[cfg(lp_io_has_gpio_matrix)]
for_each_lp_function! {
    (($_signal:ident, LP_GPIOn, $_pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl Sda for crate::peripherals::$gpio<'_> {
            fn connect_sda(&self) {
                crate::gpio::lp_io::connect_open_drain_signals(
                    self,
                    crate::gpio::lp_io::LpInputSignal::LP_I2C_SDA,
                    crate::gpio::lp_io::LpOutputSignal::LP_I2C_SDA,
                );
            }
        }

        impl Scl for crate::peripherals::$gpio<'_> {
            fn connect_scl(&self) {
                crate::gpio::lp_io::connect_open_drain_signals(
                    self,
                    crate::gpio::lp_io::LpInputSignal::LP_I2C_SCL,
                    crate::gpio::lp_io::LpOutputSignal::LP_I2C_SCL,
                );
            }
        }
    };
}

#[procmacros::doc_replace]
/// Low-power I2C driver
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = LpI2c::new(
///     peripherals.LP_I2C0,
///     Config::default(),
///     peripherals.GPIO1,
///     peripherals.GPIO2,
/// )?;
///
/// let mut data = [0u8; 22];
/// i2c.read(DEVICE_ADDR, 0xaa, &mut data)?;
/// # {after_snippet}
/// ```
pub struct LpI2c<'d> {
    i2c: LP_I2C0<'d>,
    sda: u8,
    scl: u8,
}
