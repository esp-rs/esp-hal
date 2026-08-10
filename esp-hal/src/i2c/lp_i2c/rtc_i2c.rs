//! RTC_I2C implementation of the low-power I2C driver.
//!
//! This peripheral transfers through a single data register and always needs a slave sub-register
//! address.

use crate::{
    gpio::{
        LpPin,
        lp_io::{LpFunction, low_level},
    },
    i2c::lp_i2c::{Error, LpI2c, Scl, Sda},
    peripherals::{GPIO, RTC_IO, SENS},
    time::Duration,
};

fn bind_pin(pin: &impl LpPin, function: LpFunction) {
    let lp = pin.lp_number();

    GPIO::regs()
        .pin(pin.number() as usize)
        .modify(|_, w| w.pad_driver().bit(true));
    RTC_IO::regs()
        .touch_pad(lp as usize)
        .modify(|_, w| w.rue().bit(true).rde().bit(false));
    RTC_IO::regs()
        .rtc_gpio_enable_w1ts()
        .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << lp) });
    low_level::set_config(lp, true, true, function);
}

for_each_lp_function! {
    (($_func:ident, SAR_I2C_SCL_n, $n:literal), $gpio:ident, $af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl Scl for crate::peripherals::$gpio<'_> {
            fn connect_scl(&self) {
                bind_pin(self, LpFunction::$af);
                // sar_i2c_io holds both selector fields; update only this one.
                RTC_IO::regs().sar_i2c_io().modify(|_, w| unsafe {
                    w.sar_i2c_scl_sel().bits($n)
                });
            }
        }
    };
    (($_func:ident, SAR_I2C_SDA_n, $n:literal), $gpio:ident, $af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl Sda for crate::peripherals::$gpio<'_> {
            fn connect_sda(&self) {
                bind_pin(self, LpFunction::$af);
                // sar_i2c_io holds both selector fields; update only this one.
                RTC_IO::regs().sar_i2c_io().modify(|_, w| unsafe {
                    w.sar_i2c_sda_sel().bits($n)
                });
            }
        }
    };
}

impl<'d> LpI2c<'d> {
    pub(super) fn init(&mut self) {
        // Clear any stale config registers
        self.i2c.register_block().ctrl().reset();
        SENS::regs().sar_i2c_ctrl().reset();

        // Reset RTC I2C
        SENS::regs()
            .sar_peri_reset_conf()
            .modify(|_, w| w.sar_rtc_i2c_reset().set_bit());
        self.i2c
            .register_block()
            .ctrl()
            .modify(|_, w| w.i2c_reset().set_bit());
        // The state machine does not always come out of reset when the pulse is shorter than this.
        crate::rom::ets_delay_us(20);
        self.i2c
            .register_block()
            .ctrl()
            .modify(|_, w| w.i2c_reset().clear_bit());
        SENS::regs()
            .sar_peri_reset_conf()
            .modify(|_, w| w.sar_rtc_i2c_reset().clear_bit());

        // Enable internal open-drain for SDA and SCL
        self.i2c.register_block().ctrl().modify(|_, w| {
            w.sda_force_out().clear_bit();
            w.scl_force_out().clear_bit()
        });

        // Enable clock gate.
        SENS::regs()
            .sar_peri_clk_gate_conf()
            .modify(|_, w| w.rtc_i2c_clk_en().set_bit());

        // Configure the RTC I2C controller into master mode.
        self.i2c
            .register_block()
            .ctrl()
            .modify(|_, w| w.ms_mode().set_bit());
        self.i2c
            .register_block()
            .ctrl()
            .modify(|_, w| w.i2c_ctrl_clk_gate_en().set_bit());
    }

    pub(super) fn configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let ticks = nanos_to_clock(config.timeout.as_micros().saturating_mul(1_000));

        // The register field is 20 bits wide.
        if ticks > (1 << 20) - 1 {
            return Err(ConfigError::TimeoutTooLong);
        }

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

        self.i2c
            .register_block()
            .to()
            .write(|w| unsafe { w.time_out().bits(ticks) });

        Ok(())
    }

    pub(super) fn write_bytes(
        &mut self,
        address: u8,
        register: u8,
        data: &[u8],
    ) -> Result<(), Error> {
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

    pub(super) fn read_bytes(
        &mut self,
        address: u8,
        register: u8,
        data: &mut [u8],
    ) -> Result<(), Error> {
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
            w.sar_i2c_start_force().clear_bit();
            w.sar_i2c_start().clear_bit()
        });

        result
    }

    fn clear_interrupts(&self) {
        self.i2c.register_block().int_clr().write(|w| {
            w.trans_complete().clear_bit_by_one();
            w.tx_data().clear_bit_by_one();
            w.rx_data().clear_bit_by_one();
            w.ack_err().clear_bit_by_one();
            w.time_out().clear_bit_by_one();
            w.arbitration_lost().clear_bit_by_one()
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

    pub(super) fn disable(&mut self) {
        // Reset and disable RTC I2C clock
        SENS::regs()
            .sar_peri_reset_conf()
            .modify(|_, w| w.sar_rtc_i2c_reset().set_bit());

        SENS::regs()
            .sar_peri_clk_gate_conf()
            .modify(|_, w| w.rtc_i2c_clk_en().clear_bit());
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The timeout period is longer than the configuration register allows.
    TimeoutTooLong,
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
    /// I2C timings for standard mode (100 kHz).
    pub fn standard_mode() -> Self {
        Self::default()
            .with_scl_low_period(clock_from_micros(5))
            .with_scl_high_period(clock_from_micros(5))
            .with_sda_duty(clock_from_micros(2))
            .with_scl_start_period(clock_from_micros(3))
            .with_scl_stop_period(clock_from_micros(6))
    }

    /// I2C timings for fast mode (400 kHz).
    pub fn fast_mode() -> Self {
        Self::default()
            .with_scl_low_period(clock_from_nanos(1_400))
            .with_scl_high_period(clock_from_nanos(300))
            .with_sda_duty(clock_from_nanos(1_000))
            .with_scl_start_period(clock_from_nanos(2_000))
            .with_scl_stop_period(clock_from_nanos(1_300))
    }
}

fn clock_from_micros(micros: u64) -> u32 {
    nanos_to_clock(micros * 1_000)
}

fn clock_from_nanos(nanos: u64) -> u32 {
    nanos_to_clock(nanos)
}

fn nanos_to_clock(nanos: u64) -> u32 {
    ((nanos as u128 * crate::soc::clocks::rc_fast_clk_frequency() as u128) / 1_000_000_000) as u32
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
