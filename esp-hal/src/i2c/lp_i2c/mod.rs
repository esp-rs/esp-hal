#![cfg_attr(docsrs, procmacros::doc_replace(
    "sda" => {
        cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
        _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
    },
    "scl" => {
        cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
        _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
    }
))]
//! # Low-power I2C driver
//!
//! ## Overview
//!
//! This is the host driver for the LP_I2C peripheral, which is primarily meant to be driven by
//! the ULP.
//!
//! The driver always sends a slave sub-register address along with the device address. This makes
//! it incompatible with I2C devices or sensors that do not expose sub-registers, and it also means
//! an `embedded_hal::i2c::I2c` implementation cannot be provided
//!
//! ## Configuration
//!
//! The driver can be configured using the [`Config`] struct. To create a
//! configuration, you can use the [`Config::default()`] method, and then modify
//! the individual settings as needed, by calling `with_*` methods on the
//! [`Config`] struct
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::lp_i2c::Config;
//!
//! let config = Config::default();
//! # {after_snippet}
//! ```
//!
//! You will then need to pass the configuration to [`LpI2c::new`], and you can
//! also change the configuration later by calling [`LpI2c::apply_config`]
//!
//! You will also need to specify the SDA and SCL pins when you create the
//! driver instance.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::lp_i2c::{Config, LpI2c};
//! #
//! # let config = Config::default();
//! #
//! let mut i2c = LpI2c::new(
//!     peripherals.LP_I2C0,
//!     config,
//!     peripherals.__sda__,
//!     peripherals.__scl__,
//! )?;
//!
//! // You can change the configuration later:
//! i2c.apply_config(&Config::default())?;
//! # {after_snippet}
//! ```
//!
//! ## Usage
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2c::lp_i2c::{Config, LpI2c};
//! # let mut i2c = LpI2c::new(
//! #     peripherals.LP_I2C0,
//! #     Config::default(),
//! #     peripherals.__sda__,
//! #     peripherals.__scl__,
//! # )?;
//! // The device address does not contain the `R/W` bit!
//! const DEVICE_ADDR: u8 = 0x77;
//! const DEVICE_REG: u8 = 0x01;
//! let write_buffer = [0xAA];
//! let mut read_buffer = [0u8; 22];
//!
//! i2c.write(DEVICE_ADDR, DEVICE_REG, &write_buffer)?;
//! i2c.read(DEVICE_ADDR, DEVICE_REG, &mut read_buffer)?;
//! # {after_snippet}
//! ```

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

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The transmission size exceeded the limit.
    TransactionSizeLimitExceeded,
    /// The acknowledgment check failed.
    AckCheckFailed,
    /// A timeout occurred during transmission.
    TimeOut,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
}

#[procmacros::doc_replace(
    "sda" => {
        cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
        _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
    },
    "scl" => {
        cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
        _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
    }
)]
/// Low-power I2C driver
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = LpI2c::new(
///     peripherals.LP_I2C0,
///     Config::default(),
///     peripherals.__sda__,
///     peripherals.__scl__,
/// )?;
///
/// let mut data = [0u8; 22];
/// i2c.read(DEVICE_ADDR, 0xaa, &mut data)?;
/// # {after_snippet}
/// ```
pub struct LpI2c<'d> {
    i2c: LP_I2C0<'d>,
    /// LP pin numbers, kept so that the pads can be released when the driver is dropped.
    sda: u8,
    scl: u8,
}

impl<'d> LpI2c<'d> {
    #[procmacros::doc_replace(
        "sda" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
            _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
        },
        "scl" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
            _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
        }
    )]
    /// Creates a new instance of the `LpI2c` peripheral.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
    /// let i2c = LpI2c::new(
    ///     peripherals.LP_I2C0,
    ///     Config::default(),
    ///     peripherals.__sda__,
    ///     peripherals.__scl__,
    /// )?;
    /// # {after_snippet}
    /// ```
    ///
    /// # Errors
    ///
    /// [`crate::i2c::lp_i2c::ConfigError`] when the provided config is invalid.
    pub fn new(
        i2c: LP_I2C0<'d>,
        config: Config,
        sda: impl Sda + 'd,
        scl: impl Scl + 'd,
    ) -> Result<Self, ConfigError> {
        let mut me = Self {
            i2c,
            sda: sda.lp_number(),
            scl: scl.lp_number(),
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

    #[procmacros::doc_replace(
        "sda" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
            _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
        },
        "scl" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
            _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
        }
    )]
    /// Applies a new configuration.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
    /// let mut i2c = LpI2c::new(
    ///     peripherals.LP_I2C0,
    ///     Config::default(),
    ///     peripherals.__sda__,
    ///     peripherals.__scl__,
    /// )?;
    ///
    /// i2c.apply_config(&Config::default())?;
    /// # {after_snippet}
    /// ```
    ///
    /// # Errors
    ///
    /// [`crate::i2c::lp_i2c::ConfigError`] when the provided config is invalid.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.configure(config)
    }

    #[procmacros::doc_replace(
        "sda" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
            _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
        },
        "scl" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
            _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
        }
    )]
    /// Writes `data` to the `register` of the slave with the given `address`.
    ///
    /// The transfer consists of a single write transaction that sends the device address, the
    /// register address, then `data`
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = LpI2c::new(
    ///     peripherals.LP_I2C0,
    ///     Config::default(),
    ///     peripherals.__sda__,
    ///     peripherals.__scl__,
    /// )?;
    ///
    /// i2c.write(DEVICE_ADDR, 2, &[0xaa])?;
    /// # {after_snippet}
    /// ```
    ///
    /// # Errors
    ///
    /// [`Error::TransactionSizeLimitExceeded`] when `data` is longer than the driver
    /// can transfer in a single transaction.
    pub fn write(&mut self, address: u8, register: u8, data: &[u8]) -> Result<(), Error> {
        self.write_bytes(address, register, data)
    }

    #[procmacros::doc_replace(
        "sda" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SDA_0),
            _ => gpio_for_signal!(LP_I2C_SDA, "GPIO6")
        },
        "scl" => {
            cfg(lp_i2c_master_version = "rtc_i2c") => gpio_for_signal!(SAR_I2C_SCL_1),
            _ => gpio_for_signal!(LP_I2C_SCL, "GPIO7")
        }
    )]
    /// Reads enough bytes from the `register` of the slave with the given `address` to fill
    /// `data`.
    ///
    /// The transfer writes the device address and the register address, then repeats the start
    /// condition to read `data` back
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::lp_i2c::{Config, LpI2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = LpI2c::new(
    ///     peripherals.LP_I2C0,
    ///     Config::default(),
    ///     peripherals.__sda__,
    ///     peripherals.__scl__,
    /// )?;
    ///
    /// let mut data = [0u8; 22];
    /// i2c.read(DEVICE_ADDR, 7, &mut data)?;
    /// # {after_snippet}
    /// ```
    ///
    /// # Errors
    ///
    /// [`Error::TransactionSizeLimitExceeded`] when `data` is longer than the driver
    /// can transfer in a single transaction.
    pub fn read(&mut self, address: u8, register: u8, data: &mut [u8]) -> Result<(), Error> {
        self.read_bytes(address, register, data)
    }
}

impl Drop for LpI2c<'_> {
    fn drop(&mut self) {
        self.disable();

        crate::gpio::lp_io::reset_pin(self.sda);
        crate::gpio::lp_io::reset_pin(self.scl);
    }
}
