//! LP_I2C implementation of the low-power I2C driver.
//!
//! This peripheral is a FIFO-based I2C master that executes a list of commands.

#[cfg(not(lp_io_has_gpio_matrix))]
use crate::gpio::{LpPin, lp_io::LpFunction};
#[cfg(not(esp32p4))]
use crate::peripherals::LPWR;
use crate::{
    i2c::lp_i2c::{Error, LpI2c},
    pac::lp_i2c0::{
        RegisterBlock,
        scl_high_period::{SCL_HIGH_PERIOD_SPEC, SCL_WAIT_HIGH_PERIOD_W},
        scl_low_period::{SCL_LOW_PERIOD_SPEC, SCL_LOW_PERIOD_W},
    },
    peripherals::LP_PERI,
    time::Rate,
};

const LP_I2C_FILTER_CYC_NUM_DEF: u8 = 7;

/// Depth of the TX and RX FIFOs, in bytes.
const FIFO_SIZE: usize = property!("lp_i2c_master.fifo_size");

/// Number of command slots in the command list.
const COMMAND_SLOTS: usize = 8;

#[cfg(not(lp_io_has_gpio_matrix))]
for_each_lp_function! {
    (LP_I2C_SDA, $gpio:ident, $af:ident) => {
        impl super::Sda for crate::peripherals::$gpio<'_> {
            fn connect_sda(&self) {
                configure_pad(self, LpFunction::$af);
            }
        }
    };
    (LP_I2C_SCL, $gpio:ident, $af:ident) => {
        impl super::Scl for crate::peripherals::$gpio<'_> {
            fn connect_scl(&self) {
                configure_pad(self, LpFunction::$af);
            }
        }
    };
}

enum OperationType {
    Write = 0,
    Read  = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    Ack,
    Nack,
}

#[derive(Clone, Copy)]
enum Command {
    Start,
    Stop,
    End,
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

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode: u16 = match c {
            Command::Start => 6,
            Command::Write { .. } => 1,
            Command::Stop => 2,
            Command::Read { .. } => 3,
            Command::End => 4,
        };

        let length = match c {
            Command::Start | Command::Stop | Command::End => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::End | Command::Read { .. } => Ack::Nack,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::End | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::End | Command::Write { .. } => Ack::Nack,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        }

        if ack_exp == Ack::Nack {
            cmd |= 1 << 9;
        }

        if ack_value == Ack::Nack {
            cmd |= 1 << 10;
        }

        cmd |= opcode << 11;

        cmd
    }
}

/// Configures an LP pad as an open-drain output with its pull-up enabled, then selects the pad's
/// LP I2C function.
#[cfg(not(lp_io_has_gpio_matrix))]
fn configure_pad(pin: &impl LpPin, function: LpFunction) {
    cfg_select! {
        esp32c6 => {
            use crate::peripherals::{LP_IO as LP_GPIO, LP_IO as LP_IO_MUX};
        }
        esp32c5 => {
            use crate::peripherals::{LP_GPIO, LP_IO_MUX};
        }
    }

    let ionum = pin.lp_number() as usize;

    // Set the IO pin to high to avoid them from toggling from Low to
    // High state during initialization. This can register a spurious
    // I2C start condition.
    cfg_select! {
        esp32c6 => {
            LP_GPIO::regs()
                .out_data_w1ts()
                .write(|w| unsafe { w.out_data_w1ts().bits(1 << ionum) });
        }
        esp32c5 => {
            LP_GPIO::regs()
                .out_w1ts()
                .write(|w| unsafe { w.out_w1ts().bits(1 << ionum) });
        }
    }

    // Set output mode to Open Drain
    LP_GPIO::regs()
        .pin(ionum)
        .modify(|_, w| w.pad_driver().set_bit());

    // Enable output (writing to write-1-to-set register, then internally the
    // `GPIO_OUT_REG` will be set)
    LP_GPIO::regs()
        .out_enable_w1ts()
        .write(|w| unsafe { w.enable_w1ts().bits(1 << ionum) });

    LP_IO_MUX::regs().gpio(ionum).modify(|_, w| {
        // Disable the internal weak pull-down
        w.fun_wpd().clear_bit();
        // Enable the internal weak pull-up
        w.fun_wpu().set_bit()
    });

    crate::gpio::lp_io::low_level::set_config(ionum as u8, true, true, function);
}

impl<'d> LpI2c<'d> {
    fn regs(&self) -> &RegisterBlock {
        self.i2c.register_block()
    }

    pub(super) fn init(&mut self) {
        self.enable(true);
        self.reset();
    }

    pub(super) fn configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let source_clk = crate::soc::clocks::lp_fast_clk_frequency();
        let bus_freq = config.frequency.as_hz();

        // LP I2C uses LP_FAST directly; the HP I2C clock divider does not apply.
        let half_cycle = source_clk
            .checked_div(bus_freq)
            .ok_or(ConfigError::FrequencyOutOfRange)?
            / 2;
        if !(4..(1 << SCL_LOW_PERIOD_W::<SCL_LOW_PERIOD_SPEC>::WIDTH)).contains(&half_cycle) {
            return Err(ConfigError::FrequencyOutOfRange);
        }

        // Below 80 kHz, a shorter SCL wait period avoids increasing the bus frequency.
        // See <https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_i2c/esp32c6/include/hal/i2c_ll.h#L122-L133>.
        let scl_wait_high = if bus_freq >= 80_000 {
            half_cycle / 2 - 2
        } else {
            half_cycle / 4
        };
        if scl_wait_high >= (1 << SCL_WAIT_HIGH_PERIOD_W::<SCL_HIGH_PERIOD_SPEC>::WIDTH) {
            return Err(ConfigError::FrequencyOutOfRange);
        }
        let scl_high = half_cycle - scl_wait_high;
        let sda_hold = half_cycle / 4 - 1;
        let sda_sample = half_cycle / 2 - 1;
        // Use a timeout of at least ten bus periods.
        let time_out_value = (u32::BITS - (5 * half_cycle).leading_zeros()) + 2;

        // SCL high and wait counts do not use the minus-one encoding of the other bus timings.
        // See <https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_i2c/esp32c6/include/hal/i2c_ll.h#L192-L208>.
        let period = (half_cycle - 1) as u16;

        self.select_lp_fast_clock();
        let regs = self.regs();

        regs.ctr().write(|w| unsafe {
            w.bits(0);
            // Use open-drain output for SDA and SCL.
            #[cfg(not(esp32p4))]
            {
                w.sda_force_out().set_bit();
                w.scl_force_out().set_bit();
            }
            w.clk_en().set_bit()
        });

        regs.fifo_conf().modify(|_, w| w.nonfifo_en().clear_bit());

        regs.ctr().modify(|_, w| {
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit()
        });

        self.reset_fifo();

        unsafe {
            regs.scl_low_period()
                .write(|w| w.scl_low_period().bits(period));

            regs.scl_high_period().write(|w| {
                w.scl_high_period().bits(scl_high as u16);
                w.scl_wait_high_period().bits(scl_wait_high as u8)
            });
            regs.sda_hold().write(|w| w.time().bits(sda_hold as u16));
            regs.sda_sample()
                .write(|w| w.time().bits(sda_sample as u16));

            regs.scl_rstart_setup().write(|w| w.time().bits(period));
            regs.scl_stop_setup().write(|w| w.time().bits(period));

            regs.scl_start_hold().write(|w| w.time().bits(period));
            regs.scl_stop_hold().write(|w| w.time().bits(period));

            regs.to().write(|w| {
                w.time_out_en().set_bit();
                w.time_out_value().bits(time_out_value.try_into().unwrap())
            });
        }

        regs.filter_cfg()
            .modify(|_, w| unsafe { w.sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });
        regs.filter_cfg()
            .modify(|_, w| unsafe { w.scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });

        regs.filter_cfg().modify(|_, w| w.sda_filter_en().set_bit());
        regs.filter_cfg().modify(|_, w| w.scl_filter_en().set_bit());

        // Send NACK when the RX FIFO is full.
        regs.ctr().modify(|_, w| w.rx_full_ack_level().set_bit());

        // Synchronize configuration to the LP I2C clock.
        self.lp_i2c_update();

        Ok(())
    }

    pub(super) fn write_bytes(
        &mut self,
        address: u8,
        register: u8,
        data: &[u8],
    ) -> Result<(), Error> {
        self.start_transaction();

        let mut slot = 0;
        self.write_cmd(&mut slot, Command::Start);

        self.write_fifo((address << 1) | OperationType::Write as u8);
        self.write_cmd(
            &mut slot,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1,
            },
        );

        // The register address is sent as the first payload byte.
        let payload_len = data.len() + 1;
        let payload = |index: usize| {
            if index == 0 {
                register
            } else {
                data[index - 1]
            }
        };

        // The device address takes up one FIFO slot in the first chunk.
        let mut fifo_free = FIFO_SIZE - 1;
        let mut sent = 0;

        while sent < payload_len {
            let chunk = (payload_len - sent).min(fifo_free);
            for index in sent..sent + chunk {
                self.write_fifo(payload(index));
            }
            sent += chunk;

            self.write_cmd(
                &mut slot,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: chunk as u8,
                },
            );
            // The peripheral pauses on End, so the FIFO and the command list can be refilled
            // without releasing the bus.
            self.write_cmd(
                &mut slot,
                if sent == payload_len {
                    Command::Stop
                } else {
                    Command::End
                },
            );

            self.execute()?;

            slot = 0;
            fifo_free = FIFO_SIZE;
        }

        Ok(())
    }

    pub(super) fn read_bytes(
        &mut self,
        address: u8,
        register: u8,
        data: &mut [u8],
    ) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.start_transaction();

        let mut slot = 0;

        // Select the register to read from...
        self.write_cmd(&mut slot, Command::Start);
        self.write_fifo((address << 1) | OperationType::Write as u8);
        self.write_fifo(register);
        self.write_cmd(
            &mut slot,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 2,
            },
        );

        // ... then turn the bus around with a repeated start.
        self.write_cmd(&mut slot, Command::Start);
        self.write_fifo((address << 1) | OperationType::Read as u8);
        self.write_cmd(
            &mut slot,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1,
            },
        );

        let mut received = 0;

        while received < data.len() {
            let chunk = (data.len() - received).min(FIFO_SIZE);

            if received + chunk == data.len() {
                // The slave stops sending after the last byte is NACKed.
                if chunk > 1 {
                    self.write_cmd(
                        &mut slot,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: (chunk - 1) as u8,
                        },
                    );
                }
                self.write_cmd(
                    &mut slot,
                    Command::Read {
                        ack_value: Ack::Nack,
                        length: 1,
                    },
                );
                self.write_cmd(&mut slot, Command::Stop);
            } else {
                self.write_cmd(
                    &mut slot,
                    Command::Read {
                        ack_value: Ack::Ack,
                        length: chunk as u8,
                    },
                );
                self.write_cmd(&mut slot, Command::End);
            }

            self.execute()?;

            for byte in data[received..received + chunk].iter_mut() {
                *byte = self.read_fifo();
            }
            received += chunk;

            slot = 0;
        }

        Ok(())
    }

    /// Resets the peripheral so that it can start a new transaction.
    fn start_transaction(&self) {
        // A previous transfer may have been interrupted, leaving the bus occupied.
        if self.regs().sr().read().bus_busy().bit_is_set() {
            self.regs().ctr().modify(|_, w| w.fsm_rst().set_bit());
        }

        self.reset_fifo();
        self.clear_interrupts();
    }

    /// Runs the command list and waits for the peripheral to stop.
    fn execute(&self) -> Result<(), Error> {
        self.lp_i2c_update();
        self.regs().ctr().modify(|_, w| w.trans_start().set_bit());

        let result = loop {
            let interrupts = self.regs().int_raw().read();

            if interrupts.nack().bit_is_set() {
                break Err(Error::AckCheckFailed);
            } else if interrupts.arbitration_lost().bit_is_set() {
                break Err(Error::ArbitrationLost);
            } else if interrupts.time_out().bit_is_set() {
                break Err(Error::TimeOut);
            } else if interrupts.trans_complete().bit_is_set()
                || interrupts.end_detect().bit_is_set()
            {
                break Ok(());
            }
        };

        self.clear_interrupts();

        result
    }

    fn clear_interrupts(&self) {
        self.regs().int_clr().write(|w| {
            w.nack().clear_bit_by_one();
            w.arbitration_lost().clear_bit_by_one();
            w.time_out().clear_bit_by_one();
            w.trans_complete().clear_bit_by_one();
            w.end_detect().clear_bit_by_one()
        });
    }

    fn write_cmd(&self, slot: &mut usize, command: Command) {
        debug_assert!(*slot < COMMAND_SLOTS);

        self.regs()
            .comd(*slot)
            .write(|w| unsafe { w.command().bits(command.into()) });

        *slot += 1;
    }

    fn write_fifo(&self, data: u8) {
        self.regs()
            .data()
            .write(|w| unsafe { w.fifo_rdata().bits(data) });
    }

    fn read_fifo(&self) -> u8 {
        self.regs().data().read().fifo_rdata().bits()
    }

    /// Updates I2C configuration.
    fn lp_i2c_update(&self) {
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Resets the transmit and receive FIFO buffers.
    fn reset_fifo(&self) {
        let fifo_conf = self.regs().fifo_conf();

        fifo_conf.modify(|_, w| w.tx_fifo_rst().set_bit());
        fifo_conf.modify(|_, w| w.tx_fifo_rst().clear_bit());
        fifo_conf.modify(|_, w| w.rx_fifo_rst().set_bit());
        fifo_conf.modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    fn select_lp_fast_clock(&mut self) {
        // 0 selects LP_FAST / RTC_FAST.
        cfg_select! {
            esp32p4 => {
                LP_PERI::regs()
                    .core_clk_sel()
                    .modify(|_, w| unsafe { w.lp_i2c_clk_sel().bits(0) });
            }
            _ => {
                LPWR::regs()
                    .lpperi()
                    .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());
            }
        }
    }

    pub(super) fn enable(&mut self, enable: bool) {
        let clk_en = LP_PERI::regs().clk_en();
        cfg_select! {
            esp32p4 => clk_en.modify(|_, w| w.ck_en_lp_i2c().bit(enable)),
            _ => clk_en.modify(|_, w| w.lp_ext_i2c_ck_en().bit(enable)),
        };
    }
    pub(super) fn disable(&mut self) {
        self.reset();
        self.enable(false);
    }

    pub(super) fn reset(&mut self) {
        let reset_en = LP_PERI::regs().reset_en();
        cfg_select! {
            esp32p4 => {
                reset_en.modify(|_, w| w.rst_en_lp_i2c().set_bit());
                reset_en.modify(|_, w| w.rst_en_lp_i2c().clear_bit());
            }
            _ => {
                reset_en.modify(|_, w| w.lp_ext_i2c_reset_en().set_bit());
                reset_en.modify(|_, w| w.lp_ext_i2c_reset_en().clear_bit());
            }
        }
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The requested frequency cannot be represented by the timing registers.
    FrequencyOutOfRange,
}

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C clock frequency.
    frequency: Rate,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Rate::from_khz(100),
        }
    }
}
