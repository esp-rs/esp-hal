#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # RTC I2C driver
//!
//! ## Overview
//!
//! This is the host driver for the RTC_I2C peripheral which is primarily for the ULP.
//!
//! ## Configuration
//!
//! The driver can be configured using the [`Config`] struct. To create a
//! configuration, you can use the [`Config::default()`] method, and then modify
//! the individual settings as needed, by calling `with_*` methods on the
//! [`Config`] struct.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use core::time::Duration;
//!
//! use esp_hal::i2c::rtc::Config;
//!
//! let config = Config::default().with_timeout(Duration::from_micros(100));
//! # {after_snippet}
//! ```
//!
//! You will then need to pass the configuration to [`I2c::new`], and you can
//! also change the configuration later by calling [`I2c::apply_config`].
//!
//! You will also need to specify the SDA and SCL pins when you create the
//! driver instance.
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::rtc::I2c;
//! # use core::time::Duration;
//! # use esp_hal::i2c::rtc::Config;
//! #
//! # let config = Config::default();
//! #
//! // You need to configure the driver during initialization:
//! let mut i2c = I2c::new(
//!     peripherals.RTC_I2C,
//!     config,
//!     peripherals.GPIO3,
//!     peripherals.GPIO2,
//! )?;
//!
//! // You can change the configuration later:
//! let new_config = config.with_timeout(Duration::from_micros(150));
//! i2c.apply_config(&new_config)?;
//! # {after_snippet}
//! ```
//!
//! ## Usage
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2c::rtc::{I2c, Config};
//! # let config = Config::default();
//! # let mut i2c = I2c::new(peripherals.RTC_I2C, config, peripherals.GPIO3, peripherals.GPIO2)?;
//! #
//! // `u8` is automatically converted to `I2cAddress::SevenBit`. The device
//! // address does not contain the `R/W` bit!
//! const DEVICE_ADDR: u8 = 0x77;
//! const DEVICE_REG: u8 = 0x01;
//! let write_buffer = [0xAA];
//! let mut read_buffer = [0u8; 22];
//!
//! i2c.write(DEVICE_ADDR, DEVICE_REG, &write_buffer)?;
//! i2c.read(DEVICE_ADDR, DEVICE_REG, &mut read_buffer)?;
//! # {after_snippet}
//! ```

use core::time::Duration;

use crate::{
    gpio::{InputPin, OutputPin, RtcFunction, RtcPin},
    peripherals::RTC_I2C,
};

const RC_FAST_CLK: u32 = property!("soc.rc_fast_clk_default");

/// Trait representing the RTC_I2C SDA pin.
pub trait Sda: RtcPin + OutputPin + InputPin {
    #[doc(hidden)]
    fn selector(&self) -> u8;
}

/// Trait representing the RTC_I2C SCL pin.
pub trait Scl: RtcPin + OutputPin + InputPin {
    #[doc(hidden)]
    fn selector(&self) -> u8;
}

for_each_lp_function! {
    (($_func:ident, SAR_I2C_SCL_n, $n:literal), $gpio:ident) => {
        impl Scl for crate::peripherals::$gpio<'_> {
            fn selector(&self) -> u8 {
                $n
            }
        }
    };
    (($_func:ident, SAR_I2C_SDA_n, $n:literal), $gpio:ident) => {
        impl Sda for crate::peripherals::$gpio<'_> {
            fn selector(&self) -> u8 {
                $n
            }
        }
    };
}

#[procmacros::doc_replace]
/// I2C (RTC) driver
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::i2c::rtc::{Config, I2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = I2c::new(
///     peripherals.RTC_I2C,
///     Config::default(),
///     peripherals.GPIO1,
///     peripherals.GPIO2,
/// )?;
///
/// let mut data = [0u8; 22];
/// i2c.read(DEVICE_ADDR, 0xaa, &mut data)?;
/// # {after_snippet}
/// ```
pub struct I2c<'d> {
    i2c: RTC_I2C<'d>,
}

impl<'d> I2c<'d> {
    #[procmacros::doc_replace]
    /// Create a new I2C (RTC) instance.
    ///
    /// ## Errors
    ///
    /// A [`crate::i2c::rtc::ConfigError`] variant will be returned if bus frequency or timeout
    /// passed in config is invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::rtc::{Config, I2c};
    /// let i2c = I2c::new(
    ///     peripherals.RTC_I2C,
    ///     Config::default(),
    ///     peripherals.GPIO1,
    ///     peripherals.GPIO2,
    /// )?;
    /// # {after_snippet}
    /// ```
    pub fn new(
        i2c: RTC_I2C<'d>,
        config: Config,
        sda: impl Sda + 'd,
        scl: impl Scl + 'd,
    ) -> Result<Self, ConfigError> {
        let sens = unsafe { crate::pac::SENS::steal() };
        let rtc_io = unsafe { crate::pac::RTC_IO::steal() };
        let gpio = unsafe { crate::pac::GPIO::steal() };

        // Clear any stale config registers
        i2c.register_block().ctrl().reset();
        sens.sar_i2c_ctrl().reset();

        {
            gpio.pin(scl.number() as usize)
                .modify(|_, w| w.pad_driver().bit(true));
            rtc_io
                .touch_pad(scl.number() as usize)
                .modify(|_, w| w.fun_ie().bit(true).rue().bit(true).rde().bit(false));
            rtc_io
                .rtc_gpio_enable_w1ts()
                .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << scl.number()) });
            scl.rtc_set_config(true, true, RtcFunction::I2c);
        }

        {
            gpio.pin(sda.number() as usize)
                .modify(|_, w| w.pad_driver().bit(true));
            rtc_io
                .touch_pad(sda.number() as usize)
                .modify(|_, w| w.fun_ie().bit(true).rue().bit(true).rde().bit(false));
            rtc_io
                .rtc_gpio_enable_w1ts()
                .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << sda.number()) });
            sda.rtc_set_config(true, true, RtcFunction::I2c);
        }

        rtc_io.sar_i2c_io().write(|w| unsafe {
            w.sar_i2c_sda_sel().bits(sda.selector());
            w.sar_i2c_scl_sel().bits(scl.selector())
        });

        // Reset RTC I2C
        sens.sar_peri_reset_conf()
            .modify(|_, w| w.sar_rtc_i2c_reset().set_bit());
        i2c.register_block()
            .ctrl()
            .modify(|_, w| w.i2c_reset().set_bit());
        i2c.register_block()
            .ctrl()
            .modify(|_, w| w.i2c_reset().clear_bit());
        sens.sar_peri_reset_conf()
            .modify(|_, w| w.sar_rtc_i2c_reset().clear_bit());

        // Enable internal open-drain for SDA and SCL
        i2c.register_block().ctrl().modify(|_, w| {
            w.sda_force_out().clear_bit();
            w.scl_force_out().clear_bit()
        });

        // Enable clock gate.
        sens.sar_peri_clk_gate_conf()
            .modify(|_, w| w.rtc_i2c_clk_en().set_bit());

        // Configure the RTC I2C controller into master mode.
        i2c.register_block()
            .ctrl()
            .modify(|_, w| w.ms_mode().set_bit());
        i2c.register_block()
            .ctrl()
            .modify(|_, w| w.i2c_ctrl_clk_gate_en().set_bit());

        let mut this = Self { i2c };
        this.apply_config(&config)?;
        Ok(this)
    }

    #[procmacros::doc_replace]
    /// Applies a new configuration.
    ///
    /// ## Errors
    ///
    /// A [`crate::i2c::rtc::ConfigError`] variant will be returned if bus frequency or timeout
    /// passed in config is invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use core::time::Duration;
    ///
    /// use esp_hal::i2c::rtc::{Config, I2c};
    /// let mut i2c = I2c::new(
    ///     peripherals.RTC_I2C,
    ///     Config::default(),
    ///     peripherals.GPIO1,
    ///     peripherals.GPIO2,
    /// )?;
    ///
    /// i2c.apply_config(&Config::default().with_timeout(Duration::from_micros(100)))?;
    /// # {after_snippet}
    /// ```
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.i2c
            .register_block()
            .scl_low()
            .write(|w| unsafe { w.period().bits(config.timing.scl_low_period) });
        self.i2c
            .register_block()
            .scl_high()
            .write(|w| unsafe { w.period().bits(config.timing.scl_high_period) });
        self.i2c
            .register_block()
            .sda_duty()
            .write(|w| unsafe { w.num().bits(config.timing.sda_duty) });
        self.i2c
            .register_block()
            .scl_start_period()
            .write(|w| unsafe { w.scl_start_period().bits(config.timing.scl_start_period) });
        self.i2c
            .register_block()
            .scl_stop_period()
            .write(|w| unsafe { w.scl_stop_period().bits(config.timing.scl_stop_period) });

        let ticks = duration_to_clock(config.timeout);
        let ticks = ticks.max(2u32.pow(19) - 1);
        self.i2c
            .register_block()
            .to()
            .write(|w| unsafe { w.time_out().bits(ticks) });

        Ok(())
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address` and `register`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::rtc::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(
    ///     peripherals.RTC_I2C,
    ///     Config::default(),
    ///     peripherals.GPIO1,
    ///     peripherals.GPIO2,
    /// )?;
    ///
    /// i2c.write(DEVICE_ADDR, 2, &[0xaa])?;
    /// # {after_snippet}
    /// ```
    pub fn write(&mut self, address: u8, register: u8, data: &[u8]) -> Result<(), Error> {
        let sens = unsafe { crate::pac::SENS::steal() };

        if data.len() > u8::MAX as usize - 2 {
            return Err(Error::TransactionSizeLimitExceeded);
        }

        self.write_cmd(
            0,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                // Slave addr + Reg addr + data
                length: 2 + (data.len() as u8),
            },
        );
        self.write_cmd(1, Command::Stop);

        self.clear_interrupts();

        let ctrl = {
            let mut result = 0;
            // Configure slave address.
            result |= address as u32;
            // Set slave register.
            result |= (register as u32) << 11;
            // Set first data
            result |= (data[0] as u32) << 19;
            result |= 1u32 << 27; // Write
            result
        };
        sens.sar_i2c_ctrl()
            .write(|w| unsafe { w.sar_i2c_ctrl().bits(ctrl) });

        // Start transmission.
        sens.sar_i2c_ctrl().modify(|_, w| {
            w.sar_i2c_start_force().set_bit();
            w.sar_i2c_start().set_bit()
        });

        for &byte in data.iter().skip(1) {
            match self.wait_for_tx_interrupt() {
                Ok(is_tx) => {
                    if is_tx {
                        sens.sar_i2c_ctrl().modify(|r, w| {
                            let mut value = r.sar_i2c_ctrl().bits();
                            value &= !(0xFF << 19);
                            value |= (byte as u32) << 19;
                            value |= 1 << 27;
                            unsafe { w.sar_i2c_ctrl().bits(value) }
                        });
                        self.i2c
                            .register_block()
                            .int_clr()
                            .write(|w| w.tx_data().clear_bit_by_one());
                    } else {
                        core::panic!("Peripheral didn't wait for data");
                    }
                }
                Err(err) => {
                    // Stop transmission.
                    sens.sar_i2c_ctrl().modify(|_, w| {
                        w.sar_i2c_start_force().clear_bit();
                        w.sar_i2c_start().clear_bit()
                    });

                    return Err(err);
                }
            }
        }

        let result = self.wait_for_complete_interrupt();

        // Stop transmission.
        sens.sar_i2c_ctrl().write(|w| {
            w.sar_i2c_start_force().clear_bit();
            w.sar_i2c_start().clear_bit()
        });

        result
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address` and `register`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::rtc::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(
    ///     peripherals.RTC_I2C,
    ///     Config::default(),
    ///     peripherals.GPIO1,
    ///     peripherals.GPIO2,
    /// )?;
    ///
    /// let mut data = [0u8; 22];
    /// i2c.read(DEVICE_ADDR, 7, &mut data)?;
    /// # {after_snippet}
    /// ```
    pub fn read(&mut self, address: u8, register: u8, data: &mut [u8]) -> Result<(), Error> {
        let sens = unsafe { crate::pac::SENS::steal() };

        if data.len() > u8::MAX as usize {
            return Err(Error::TransactionSizeLimitExceeded);
        }

        // Slave addr + Reg addr
        self.write_cmd(
            2,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 2,
            },
        );
        // Restart
        self.write_cmd(3, Command::Start);
        self.write_cmd(
            4,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                // Reg addr
                length: 1,
            },
        );
        if data.len() > 1 {
            self.write_cmd(
                5,
                Command::Read {
                    ack_value: Ack::Ack,
                    length: (data.len() - 1) as _,
                },
            );
            self.write_cmd(
                6,
                Command::Read {
                    ack_value: Ack::Nack,
                    length: 1,
                },
            );
            self.write_cmd(7, Command::Stop);
        } else {
            self.write_cmd(
                5,
                Command::Read {
                    ack_value: Ack::Nack,
                    length: 1,
                },
            );
            self.write_cmd(6, Command::Stop);
        }

        self.clear_interrupts();

        // Start transmission.
        let ctrl = {
            let mut result = 0;
            result |= address as u32;
            result |= (register as u32) << 11;
            result |= 0u32 << 27; // Read
            result
        };
        sens.sar_i2c_ctrl().write(|w| {
            unsafe { w.sar_i2c_ctrl().bits(ctrl) };
            w.sar_i2c_start_force().set_bit();
            w.sar_i2c_start().set_bit()
        });

        for byte in data {
            match self.wait_for_rx_interrupt() {
                Ok(is_rx) => {
                    if is_rx {
                        *byte = self.i2c.register_block().data().read().i2c_rdata().bits();
                        self.i2c
                            .register_block()
                            .int_clr()
                            .write(|w| w.rx_data().clear_bit_by_one());
                    } else {
                        core::panic!("Peripheral didn't wait for data to be read");
                    }
                }
                Err(err) => {
                    // Stop transmission.
                    sens.sar_i2c_ctrl().modify(|_, w| {
                        w.sar_i2c_start_force().clear_bit();
                        w.sar_i2c_start().clear_bit()
                    });

                    return Err(err);
                }
            }
        }

        let result = self.wait_for_complete_interrupt();

        // Stop transmission.
        sens.sar_i2c_ctrl().modify(|_, w| {
            w.sar_i2c_start_force()
                .clear_bit()
                .sar_i2c_start()
                .clear_bit()
        });

        result
    }

    fn clear_interrupts(&self) {
        self.i2c.register_block().int_clr().write(|w| {
            w.trans_complete()
                .clear_bit_by_one()
                .tx_data()
                .clear_bit_by_one()
                .rx_data()
                .clear_bit_by_one()
                .ack_err()
                .clear_bit_by_one()
                .time_out()
                .clear_bit_by_one()
                .arbitration_lost()
                .clear_bit_by_one()
        });
    }

    fn wait_for_tx_interrupt(&self) -> Result<bool, Error> {
        loop {
            let int_raw = self.i2c.register_block().int_raw().read();
            if int_raw.tx_data().bit_is_set() {
                break Ok(true);
            } else if int_raw.trans_complete().bit_is_set() {
                break Ok(false);
            } else if int_raw.time_out().bit_is_set() {
                break Err(Error::TimeOut);
            } else if int_raw.ack_err().bit_is_set() {
                break Err(Error::AckCheckFailed);
            } else if int_raw.arbitration_lost().bit_is_set() {
                break Err(Error::ArbitrationLost);
            }
        }
    }

    fn wait_for_rx_interrupt(&self) -> Result<bool, Error> {
        loop {
            let int_raw = self.i2c.register_block().int_raw().read();
            if int_raw.rx_data().bit_is_set() {
                break Ok(true);
            } else if int_raw.trans_complete().bit_is_set() {
                break Ok(false);
            } else if int_raw.time_out().bit_is_set() {
                break Err(Error::TimeOut);
            } else if int_raw.ack_err().bit_is_set() {
                break Err(Error::AckCheckFailed);
            } else if int_raw.arbitration_lost().bit_is_set() {
                break Err(Error::ArbitrationLost);
            }
        }
    }

    fn wait_for_complete_interrupt(&self) -> Result<(), Error> {
        loop {
            let int_raw = self.i2c.register_block().int_raw().read();
            if int_raw.trans_complete().bit_is_set() {
                break Ok(());
            } else if int_raw.time_out().bit_is_set() {
                break Err(Error::TimeOut);
            } else if int_raw.ack_err().bit_is_set() {
                break Err(Error::AckCheckFailed);
            } else if int_raw.arbitration_lost().bit_is_set() {
                break Err(Error::ArbitrationLost);
            }
        }
    }

    fn write_cmd(&self, idx: usize, command: Command) {
        let cmd = command.into();
        self.i2c
            .register_block()
            .cmd(idx)
            .write(|w| unsafe { w.command().bits(cmd) });
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {}

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

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C timings (clock frequency).
    timing: Timing,

    /// I2C SCL timeout period.
    timeout: Duration,
}

/// I2C timings
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    /// SCL low period
    scl_low_period: u32,
    /// SCL high period
    scl_high_period: u32,
    /// Period between the SDA switch and the falling edge of SCL
    sda_duty: u32,
    /// Waiting time after the START condition in micro seconds
    scl_start_period: u32,
    /// Waiting time before the END condition in micro seconds
    scl_stop_period: u32,
}

impl Timing {
    /// I2C timing from durations.
    pub const fn from_config(
        scl_low_period: Duration,
        scl_high_period: Duration,
        sda_duty: Duration,
        scl_start_period: Duration,
        scl_stop_period: Duration,
    ) -> Self {
        Self {
            scl_low_period: duration_to_clock(scl_low_period),
            scl_high_period: duration_to_clock(scl_high_period),
            sda_duty: duration_to_clock(sda_duty),
            scl_start_period: duration_to_clock(scl_start_period),
            scl_stop_period: duration_to_clock(scl_stop_period),
        }
    }

    /// I2C timings for standard mode (100 kHz).
    pub const fn standard_mode() -> Self {
        Self::from_config(
            Duration::from_micros(5),
            Duration::from_micros(5),
            Duration::from_micros(2),
            Duration::from_micros(3),
            Duration::from_micros(6),
        )
    }

    /// I2C timings for fast mode (400 kHz).
    pub const fn fast_mode() -> Self {
        Self::from_config(
            Duration::from_nanos(1_400),
            Duration::from_nanos(300),
            Duration::from_nanos(1_000),
            Duration::from_nanos(2_000),
            Duration::from_nanos(1_300),
        )
    }
}

const fn duration_to_clock(value: Duration) -> u32 {
    ((value.as_nanos() * RC_FAST_CLK as u128) / 1_000_000_000) as u32
}

/// A generic I2C Command
enum Command {
    Start,
    Stop,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp
        /// value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is
        /// 255, while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte
        /// has been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    Ack,
    Nack,
}

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode = match c {
            Command::Start => 0,
            Command::Stop => 3,
            Command::Write { .. } => 1,
            Command::Read { .. } => 2,
        };

        let length = match c {
            Command::Start | Command::Stop => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::Read { .. } => Ack::Nack,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::Write { .. } => Ack::Nack,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        } else {
            cmd &= !(1 << 8);
        }

        if ack_exp == Ack::Nack {
            cmd |= 1 << 9;
        } else {
            cmd &= !(1 << 9);
        }

        if ack_value == Ack::Nack {
            cmd |= 1 << 10;
        } else {
            cmd &= !(1 << 10);
        }

        cmd |= opcode << 11;

        cmd
    }
}
