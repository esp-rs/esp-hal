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
    peripherals::{RTC_IO, SENS},
    system::{Peripheral, PeripheralClockControl},
    time::Duration,
};

fn bind_pin(pin: &impl LpPin, function: LpFunction) {
    let lp = pin.lp_number();
    let rtc_io = RTC_IO::regs();

    rtc_io
        .pin(lp as usize)
        .modify(|_, w| w.pad_driver().set_bit());
    rtc_io.touch_pad(lp as usize).modify(|_, w| {
        w.rue().set_bit();
        w.rde().clear_bit()
    });
    low_level::output_enable(lp, true);
    low_level::set_config(lp, true, true, function);
}

for_each_lp_function! {
    (($_func:ident, SAR_I2C_SCL_n, $n:literal), $gpio:ident, $af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl Scl for crate::peripherals::$gpio<'_> {
            fn connect_scl(&self) {
                bind_pin(self, LpFunction::$af);
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
                RTC_IO::regs().sar_i2c_io().modify(|_, w| unsafe {
                    w.sar_i2c_sda_sel().bits($n)
                });
            }
        }
    };
}

impl<'d> LpI2c<'d> {
    pub(super) fn init(&mut self) {
        self.i2c.register_block().ctrl().reset();
        SENS::regs().sar_i2c_ctrl().reset();

        PeripheralClockControl::set_reset(Peripheral::RtcI2c, true);
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
        PeripheralClockControl::set_reset(Peripheral::RtcI2c, false);

        // Enable internal open-drain for SDA and SCL
        self.i2c.register_block().ctrl().modify(|_, w| {
            w.sda_force_out().clear_bit();
            w.scl_force_out().clear_bit()
        });

        PeripheralClockControl::set_enabled_untracked(Peripheral::RtcI2c, true);

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
        use crate::pac::rtc_i2c::{
            scl_low::{PERIOD_W as LOW_W, SCL_LOW_SPEC},
            scl_start_period::{SCL_START_PERIOD_SPEC, SCL_START_PERIOD_W},
            scl_stop_period::{SCL_STOP_PERIOD_SPEC, SCL_STOP_PERIOD_W},
            to::{TIME_OUT_W, TO_SPEC},
        };

        let timing = &config.timing;
        // SDA duty and SCL high have the same width as their upper bounds below.
        if timing.scl_low_period >= (1 << LOW_W::<SCL_LOW_SPEC>::WIDTH)
            || timing.scl_start_period >= (1 << SCL_START_PERIOD_W::<SCL_START_PERIOD_SPEC>::WIDTH)
            || timing.scl_stop_period >= (1 << SCL_STOP_PERIOD_W::<SCL_STOP_PERIOD_SPEC>::WIDTH)
        {
            return Err(ConfigError::TimingOutOfRange);
        }

        // The RTC input filters require minimum sample windows and ordered timing edges.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/ulp/ulp_riscv/ulp_riscv_i2c.c#L174-L180>.
        if timing.scl_stop_period <= timing.scl_high_period
            || timing.sda_duty >= timing.scl_low_period
            || timing.scl_start_period <= 8
            || timing.sda_duty <= 14
            || timing.scl_low_period - timing.sda_duty + timing.scl_high_period <= 8
        {
            return Err(ConfigError::TimingOutOfRange);
        }

        let ticks = config.timeout.as_micros() as u128
            * crate::soc::clocks::rtc_fast_clk_frequency() as u128
            / 1_000_000;
        if ticks >= (1 << TIME_OUT_W::<TO_SPEC>::WIDTH) {
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
            .write(|w| unsafe { w.time_out().bits(ticks as u32) });

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
            result |= (data.first().copied().unwrap_or(0) as u32) << 19;
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
                Ok(true) => {
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
                }
                Ok(false) => panic!("Peripheral didn't wait for data"),
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
                Ok(true) => {
                    *byte = self.i2c.register_block().data().read().i2c_rdata().bits();
                    self.i2c
                        .register_block()
                        .int_clr()
                        .write(|w| w.rx_data().clear_bit_by_one());
                }
                Ok(false) => panic!("Peripheral didn't wait for data to be read"),
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
        PeripheralClockControl::set_reset(Peripheral::RtcI2c, true);
        PeripheralClockControl::set_enabled_untracked(Peripheral::RtcI2c, false);
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// A timing value exceeds a register field or violates a hardware timing constraint.
    TimingOutOfRange,
    /// The timeout period is longer than the configuration register allows.
    TimeoutTooLong,
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TimingOutOfRange => f.write_str(
                "The timing values exceed a register field or violate a hardware constraint",
            ),
            Self::TimeoutTooLong => {
                f.write_str("The timeout exceeds its configuration register field")
            }
        }
    }
}

impl core::error::Error for ConfigError {}

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C timings. Defaults to the standard-mode preset.
    timing: Timing,

    /// I2C SCL timeout period. Defaults to 20 microseconds.
    timeout: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            timing: Timing::default(),
            timeout: Duration::from_micros(20),
        }
    }
}

/// I2C timings. Defaults to the standard-mode preset.
///
/// Presets use the RTC_FAST clock frequency when they are constructed.
/// At low frequencies, the presets lengthen their periods to
/// keep SDA duty at or above the hardware minimum of 15 cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    /// SCL low period, in RTC_FAST clock cycles.
    scl_low_period: u32,
    /// SCL high period, in RTC_FAST clock cycles.
    scl_high_period: u32,
    /// Period between the SDA switch and the falling edge of SCL, in RTC_FAST clock cycles.
    sda_duty: u32,
    /// Waiting time after the START condition, in RTC_FAST clock cycles.
    scl_start_period: u32,
    /// Waiting time before the STOP condition, in RTC_FAST clock cycles.
    scl_stop_period: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self::standard_mode()
    }
}

impl Timing {
    /// Returns I2C timings for nominal standard mode (100 kHz).
    pub fn standard_mode() -> Self {
        let frequency = preset_clock_frequency().max(7_500_000);
        Self {
            scl_low_period: clock_from_nanos(5_000, frequency),
            scl_high_period: clock_from_nanos(5_000, frequency),
            sda_duty: clock_from_nanos(2_000, frequency),
            scl_start_period: clock_from_nanos(3_000, frequency),
            scl_stop_period: clock_from_nanos(6_000, frequency),
        }
    }

    /// Returns a timing preset targeting 400 kHz.
    ///
    /// With an RTC_FAST clock of 15 MHz or higher, the configured SCL high
    /// period is about 0.3 µs, below the Fast-mode minimum of 0.6 µs.
    /// The actual SCL timing depends on the RTC_FAST clock and the bus rise time.
    pub fn fast_mode() -> Self {
        let frequency = preset_clock_frequency().max(15_000_000);
        Self {
            scl_low_period: clock_from_nanos(1_400, frequency),
            scl_high_period: clock_from_nanos(300, frequency),
            sda_duty: clock_from_nanos(1_000, frequency),
            scl_start_period: clock_from_nanos(2_000, frequency),
            scl_stop_period: clock_from_nanos(1_300, frequency),
        }
    }
}

fn preset_clock_frequency() -> u32 {
    match crate::soc::clocks::rtc_fast_clk_frequency() {
        // RTC_FAST has no configured frequency before HAL initialization.
        0 => crate::soc::clocks::rc_fast_clk_frequency(),
        frequency => frequency,
    }
}

fn clock_from_nanos(nanos: u32, frequency: u32) -> u32 {
    (u64::from(nanos) * u64::from(frequency) / 1_000_000_000) as u32
}

/// A generic I2C Command.
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
