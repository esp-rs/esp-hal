//! LP_I2C implementation of the low-power I2C driver.
//!
//! FIFO-based I2C master that executes a list of commands.

#[cfg(not(lp_io_has_gpio_matrix))]
use crate::gpio::{LpPin, lp_io::LpFunction};
use crate::{
    i2c::lp_i2c::{Error, LpI2c, Scl, Sda},
    pac::lp_i2c0::RegisterBlock,
    peripherals::{LP_PERI, LPWR},
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
        impl Sda for crate::peripherals::$gpio<'_> {
            fn connect_sda(&self) {
                configure_pad(self, LpFunction::$af);
            }
        }
    };
    (LP_I2C_SCL, $gpio:ident, $af:ident) => {
        impl Scl for crate::peripherals::$gpio<'_> {
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
        /// Expected ACK value for the transmitter.
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

// https://github.com/espressif/esp-idf/blob/master/components/ulp/lp_core/lp_core_i2c.c#L122
// TX/RX RAM size is 16*8 bit
// TX RX FIFO has 16 bit depth
// The clock source of APB_CLK in LP_I2C is CLK_AON_FAST.
// Configure LP_I2C_SCLK_SEL to select the clock source for I2C_SCLK.
// When LP_I2C_SCLK_SEL is 0, select CLK_ROOT_FAST as clock source,
// and when LP_I2C_SCLK_SEL is 1, select CLK _XTALD2 as the clock source.
// Configure LP_EXT_I2C_CK_EN high to enable the clock source of I2C_SCLK.
// Adjust the timing registers accordingly when the clock frequency changes.

/// Configures an LP pad as an open-drain output with its pull-up enabled, then selects the pad's
/// LP I2C function.
#[cfg(not(lp_io_has_gpio_matrix))]
fn configure_pad(pin: &impl LpPin, function: LpFunction) {
    use crate::peripherals::LP_IO;

    let ionum = pin.lp_number() as usize;
    let lp_io = LP_IO::regs();
    unsafe {
        // Set the IO pin to high to avoid them from toggling from Low to
        // High state during initialization. This can register a spurious
        // I2C start condition.
        lp_io
            .out_data_w1ts()
            .write(|w| w.out_data_w1ts().bits(1 << ionum));

        // Set output mode to Open Drain
        lp_io.pin(ionum).modify(|_, w| w.pad_driver().set_bit());

        // Enable output (writing to write-1-to-set register, then internally the
        // `GPIO_OUT_REG` will be set)
        lp_io
            .out_enable_w1ts()
            .write(|w| w.enable_w1ts().bits(1 << ionum));

        lp_io.gpio(ionum).modify(|_, w| {
            // Disable the internal weak pull-down
            w.fun_wpd().clear_bit();
            // Enable the internal weak pull-up
            w.fun_wpu().set_bit()
        });
    }

    crate::gpio::lp_io::low_level::set_config(ionum as u8, true, true, function);
}

impl<'d> LpI2c<'d> {
    fn regs(&self) -> &RegisterBlock {
        self.i2c.register_block()
    }

    pub(super) fn init(&mut self) {
        // Initialize LP I2C HAL */
        self.i2c
            .register_block()
            .clk_conf()
            .modify(|_, w| w.sclk_active().set_bit());

        // Enable LP I2C controller clock
        LP_PERI::regs()
            .clk_en()
            .modify(|_, w| w.lp_ext_i2c_ck_en().set_bit());

        LP_PERI::regs()
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().set_bit());
        LP_PERI::regs()
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().clear_bit());
    }

    pub(super) fn configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Set LP I2C source clock
        LPWR::regs()
            .lpperi()
            .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

        // Initialize LP I2C Master mode
        self.i2c.register_block().ctr().modify(|_, w| unsafe {
            // Clear register
            w.bits(0);
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });

        // First, reset the fifo buffers
        self.i2c
            .register_block()
            .fifo_conf()
            .modify(|_, w| w.nonfifo_en().clear_bit());

        self.i2c.register_block().ctr().modify(|_, w| {
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit()
        });

        self.reset_fifo();

        // Set LP I2C source clock
        LPWR::regs()
            .lpperi()
            .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

        // Configure LP I2C timing paramters. source_clk is ignored for LP_I2C in this
        // call

        let source_clk = 16_000_000;
        let bus_freq = config.frequency.as_hz();

        let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
        let sclk_freq: u32 = source_clk / clkm_div;
        let half_cycle: u32 = sclk_freq / bus_freq / 2;

        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        // Make 80KHz as a boundary here, because when working at lower frequency, too
        // much scl_wait_high will faster the frequency according to some
        // hardware behaviors.
        let scl_wait_high = if bus_freq >= 80 * 1000 {
            half_cycle / 2 - 2
        } else {
            half_cycle / 4
        };
        let scl_high = half_cycle - scl_wait_high;
        let sda_hold = half_cycle / 4;
        let sda_sample = half_cycle / 2; // TODO + scl_wait_high;
        let setup = half_cycle;
        let hold = half_cycle;
        // default we set the timeout value to about 10 bus cycles
        // log(20*half_cycle)/log(2) = log(half_cycle)/log(2) +  log(20)/log(2)
        let tout = (4 * 8 - (5 * half_cycle).leading_zeros()) + 2;

        // According to the Technical Reference Manual, the following timings must be
        // subtracted by 1. However, according to the practical measurement and
        // some hardware behaviour, if wait_high_period and scl_high minus one.
        // The SCL frequency would be a little higher than expected. Therefore, the
        // solution here is not to minus scl_high as well as scl_wait high, and
        // the frequency will be absolutely accurate to all frequency
        // to some extent.
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold - 1;
        let sda_sample_time = sda_sample - 1;
        // setup
        let scl_rstart_setup_time = setup - 1;
        let scl_stop_setup_time = setup - 1;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold - 1;
        let time_out_value = tout;
        let time_out_en = true;

        // Write data to registers
        unsafe {
            self.i2c.register_block().clk_conf().modify(|_, w| {
                w.sclk_sel().clear_bit();
                w.sclk_div_num().bits((clkm_div - 1) as u8)
            });

            // scl period
            self.i2c
                .register_block()
                .scl_low_period()
                .write(|w| w.scl_low_period().bits(scl_low_period as u16));

            self.i2c.register_block().scl_high_period().write(|w| {
                w.scl_high_period().bits(scl_high_period as u16);
                w.scl_wait_high_period().bits(scl_wait_high_period as u8)
            });
            // sda sample
            self.i2c
                .register_block()
                .sda_hold()
                .write(|w| w.time().bits(sda_hold_time as u16));
            self.i2c
                .register_block()
                .sda_sample()
                .write(|w| w.time().bits(sda_sample_time as u16));

            // setup
            self.i2c
                .register_block()
                .scl_rstart_setup()
                .write(|w| w.time().bits(scl_rstart_setup_time as u16));
            self.i2c
                .register_block()
                .scl_stop_setup()
                .write(|w| w.time().bits(scl_stop_setup_time as u16));

            // hold
            self.i2c
                .register_block()
                .scl_start_hold()
                .write(|w| w.time().bits(scl_start_hold_time as u16));
            self.i2c
                .register_block()
                .scl_stop_hold()
                .write(|w| w.time().bits(scl_stop_hold_time as u16));

            self.i2c.register_block().to().write(|w| {
                w.time_out_en().bit(time_out_en);
                w.time_out_value().bits(time_out_value.try_into().unwrap())
            });
        }

        // Enable SDA and SCL filtering. This configuration matches the HP I2C filter
        // config

        self.i2c
            .register_block()
            .filter_cfg()
            .modify(|_, w| unsafe { w.sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });
        self.i2c
            .register_block()
            .filter_cfg()
            .modify(|_, w| unsafe { w.scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });

        self.i2c
            .register_block()
            .filter_cfg()
            .modify(|_, w| w.sda_filter_en().set_bit());
        self.i2c
            .register_block()
            .filter_cfg()
            .modify(|_, w| w.scl_filter_en().set_bit());

        // Configure the I2C master to send a NACK when the Rx FIFO count is full
        self.i2c
            .register_block()
            .ctr()
            .modify(|_, w| w.rx_full_ack_level().set_bit());

        // Synchronize the config register values to the LP I2C peripheral clock
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

    /// Update I2C configuration
    fn lp_i2c_update(&self) {
        self.i2c
            .register_block()
            .ctr()
            .modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Resets the transmit and receive FIFO buffers.
    fn reset_fifo(&self) {
        self.i2c
            .register_block()
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().set_bit());

        self.i2c
            .register_block()
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit());

        self.i2c
            .register_block()
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().set_bit());

        self.i2c
            .register_block()
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    pub(super) fn disable(&mut self) {
        // Reset the peripheral so that it stops driving the bus, then take away its clocks.
        LP_PERI::regs()
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().set_bit());
        LP_PERI::regs()
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().clear_bit());

        LP_PERI::regs()
            .clk_en()
            .modify(|_, w| w.lp_ext_i2c_ck_en().clear_bit());
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {}

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
