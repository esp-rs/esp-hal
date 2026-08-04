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
    #[allow(unused)]
    sda: u8,
    #[allow(unused)]
    scl: u8,
}

impl<'d> LpI2c<'d> {
    /// Creates a new instance of the `LpI2c` peripheral.
    ///
    /// ## Errors
    ///
    /// A [`crate::i2c::lp_i2c::ConfigError`] variant will be returned if the provided config is
    /// invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
    /// let i2c = LpI2c::new(
    ///     peripherals.LP_I2C0,
    ///     Config::default(),
    ///     peripherals.GPIO1,
    ///     peripherals.GPIO2,
    /// )?;
    /// # {after_snippet}
    /// ```
    pub fn new(
        i2c: LP_I2C0<'d>,
        config: Config,
        sda: impl Sda + 'd,
        scl: impl Scl + 'd,
    ) -> Result<Self, ConfigError> {
        let mut me = Self {
            i2c,
            sda: sda.number(),
            scl: scl.number(),
        };

        me.init();

        // Configure LP I2C GPIOs
        // NOTE: We always initialize the SCL pin first, then the SDA pin. This order of
        // initialization is important to avoid any spurious I2C start conditions on the bus.
        scl.connect_scl();
        sda.connect_sda();

        me.apply_config(&config)?;

        Ok(me)
    }
}

impl Drop for LpI2c<'_> {
    fn drop(&mut self) {
        self.disable();
    }
}
