//! I2C Driver
//!
//! Supports multiple I2C peripheral instances
use core::convert::TryInto;

use crate::gpio::{InputPin, OutputPin};
#[cfg(feature = "esp32c3")]
use crate::pac::i2c::{RegisterBlock, COMD};
#[cfg(not(feature = "esp32c3"))]
use crate::pac::i2c0::{RegisterBlock, COMD};

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c3")] {
        const SOURCE_CLK_FREQ: u32 = 20_000_000;
    }
    else {
        const SOURCE_CLK_FREQ: u32 = 40_000_000;
    }
}

/// I2C-specific transmission errors
#[derive(Debug)]
pub enum Error {
    ExceedingFifo,
    AckCheckFailed,
    TimeOut,
    ArbitrationLost,
    ExecIncomplete,
    CommandNrExceeded,
}

/// I2C-specific setup errors
#[derive(Debug)]
pub enum SetupError {
    InvalidClkConfig,
    PeripheralDisabled,
}

#[macro_export]
macro_rules! i2c_peripheral {
    (
        $output_signal_type:ty,
        $input_signal_type:ty,

        $(
            $peripheral:ident:
            (
                $sda_output_signal:path,
                $sda_input_signal:path,
                $scl_output_signal:path,
                $scl_input_signal:path
            )
        )+
    ) => {
        use embedded_hal::blocking::i2c::*;
        use esp_hal_common::{
            gpio::{InputPin, OutputPin},
            i2c::Instance,
        };

        /// I2C peripheral container (I2C)
        pub struct I2C<T> {
            peripheral: T,
        }

        impl<T> Read for I2C<T>
        where I2C<T>: Instance
        {
            type Error = Error;

            fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                self.master_read(address, buffer)
            }
        }

        impl<T> Write for I2C<T>
        where I2C<T>: Instance
        {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                self.master_write(addr, bytes)
            }
        }

        impl<T> WriteRead for I2C<T>
        where I2C<T>: Instance
        {
            type Error = Error;

            fn write_read(
                &mut self,
                address: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                self.master_write_read(address, bytes, buffer)
            }
        }

        $(
            impl I2C<$peripheral> {
                /// Create a new I2C instance
                pub fn new<
                    SDA: OutputPin<OutputSignal = $output_signal_type>
                        + InputPin<InputSignal = $input_signal_type>,
                    SCL: OutputPin<OutputSignal = $output_signal_type>
                        + InputPin<InputSignal = $input_signal_type>,
                >(
                    instance: $peripheral,
                    mut pins: Pins<SDA, SCL>,
                    frequency: u32,
                ) -> Result<Self, SetupError> {
                    let mut i2c = I2C {
                        peripheral: instance,
                    };

                    pins.sda
                        .set_to_open_drain_output()
                        .enable_input(true)
                        .internal_pull_up(true)
                        .connect_peripheral_to_output($sda_output_signal)
                        .connect_input_to_peripheral($sda_input_signal);

                    pins.scl
                        .set_to_open_drain_output()
                        .enable_input(true)
                        .internal_pull_up(true)
                        .connect_peripheral_to_output($scl_output_signal)
                        .connect_input_to_peripheral($scl_input_signal);

                    i2c.setup(frequency)?;

                    Ok(i2c)
                }

                /// Return the raw interface to the underlying peripheral
                pub fn free(self) -> $peripheral {
                    self.peripheral
                }
            }

            impl Instance for I2C<$peripheral> {
                #[inline(always)]
                fn register_block(&self) -> &RegisterBlock {
                    &self.peripheral
                }
            }
        )+
    };
}

/// I2C Peripheral Instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn setup(&mut self, frequency: u32) -> Result<(), SetupError> {
        // Reset entire peripheral (also resets fifo)
        self.reset();

        self.register_block().ctr.modify(|_, w| unsafe {
            // Clear register
            w.bits(0)
                // Set I2C controller to master mode
                .ms_mode()
                .set_bit()
                // Use open drain output for SDA and SCL
                .sda_force_out()
                .set_bit()
                .scl_force_out()
                .set_bit()
                // Use Most Significant Bit first for sending and receiving data
                .tx_lsb_first()
                .clear_bit()
                .rx_lsb_first()
                .clear_bit()
                // Ensure that clock is enabled
                .clk_en()
                .set_bit()
        });

        // Configure filter
        self.set_filter(Some(7), Some(7));

        // Configure frequency
        self.set_frequency(SOURCE_CLK_FREQ, frequency)?;

        // Propagate configuration changes (only necessary with C3)
        #[cfg(feature = "esp32c3")]
        self.register_block()
            .ctr
            .modify(|_, w| w.conf_upgate().set_bit());

        Ok(())
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    fn reset(&mut self) {
        // Reset interrupts
        // Disable all I2C interrupts
        self.register_block()
            .int_ena
            .write(|w| unsafe { w.bits(0) });
        // Clear all I2C interrupts
        self.register_block()
            .int_clr
            .write(|w| unsafe { w.bits(0x3FFF) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();

        // Reset the FSM
        // (the option to reset the FSM is not available
        // for the ESP32)
        #[cfg(not(feature = "esp32"))]
        self.register_block()
            .ctr
            .modify(|_, w| w.fsm_rst().set_bit());
    }

    /// Resets the I2C peripheral's command registers
    fn reset_command_list(&mut self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.register_block().comd.iter() {
            cmd.reset();
        }
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a
    /// pulse must be present to pass the filter
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let sda_register = &self.register_block().sda_filter_cfg;
                let scl_register = &self.register_block().scl_filter_cfg;
            } else {
                let sda_register = &self.register_block().filter_cfg;
                let scl_register = &self.register_block().filter_cfg;
            }
        }

        match sda_threshold {
            Some(threshold) => {
                sda_register.modify(|_, w| unsafe { w.sda_filter_thres().bits(threshold) });
                sda_register.modify(|_, w| w.sda_filter_en().set_bit());
            }
            None => sda_register.modify(|_, w| w.sda_filter_en().clear_bit()),
        }

        match scl_threshold {
            Some(threshold) => {
                scl_register.modify(|_, w| unsafe { w.scl_filter_thres().bits(threshold) });
                scl_register.modify(|_, w| w.scl_filter_en().set_bit());
            }
            None => scl_register.modify(|_, w| w.scl_filter_en().clear_bit()),
        }
    }

    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings
    fn set_frequency(&mut self, source_clk: u32, bus_freq: u32) -> Result<(), SetupError> {
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32s3", feature = "esp32c3"))] {
                // C3 and S3 have a clock devider mechanism, which we want to configure
                // as high as possible.
                let sclk_div: u8 = (source_clk / (bus_freq * 1024) + 1)
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;

                let half_cycle: u16 = (source_clk / (bus_freq * (sclk_div as u32) * 2) as u32)
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
            } else {
                // For EPS32 and the S2 variant no clock divider mechanism exists.
                let half_cycle: u16 = (source_clk / (bus_freq * 2) as u32)
                .try_into()
                .map_err(|_| SetupError::InvalidClkConfig)?;
            }
        }

        // The different chips have highly very different timing configurations, so
        // we're setting these up separately (this might introduce someoverhead,
        // but improves readability)
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32c3")] {
                let scl_wait_high: u8 = (if bus_freq <= 50000 { 0 } else { half_cycle / 8 })
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
                let scl_high: u16 = half_cycle - scl_wait_high as u16;
                let sda_hold = half_cycle / 4;
                let sda_sample = scl_high / 2;
            }
            else if #[cfg(feature = "esp32s3")] {
                let scl_high: u8 = (if bus_freq <= 50000 { half_cycle } else { (half_cycle / 5 * 4 + 4) })
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
                let scl_wait_high = half_cycle - scl_high;
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2;
            }
            else if #[cfg(feature = "esp32s2")] {
                let scl_high = half_cycle / 2 + 2;
                let scl_wait_high = half_cycle - scl_high;
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2 - 1;
            }
            else {
                // ESP32 is default (as it is the simplest case and does not even have
                // the wait_high field)
                let scl_high = half_cycle;
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2;
                let tout: u16 = (half_cycle * 20)
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
                    }
        }

        let scl_low = half_cycle;
        let setup = half_cycle;
        let hold = half_cycle;

        unsafe {
            // divider
            #[cfg(any(feature = "esp32s3", feature = "esp32c3"))]
            self.register_block()
                .clk_conf
                .modify(|_, w| w.sclk_sel().clear_bit().sclk_div_num().bits(sclk_div));

            // scl period
            self.register_block()
                .scl_low_period
                .write(|w| w.scl_low_period().bits(scl_low));

            // for high/wait_high we have to differentiate between the chips
            // as the EPS32 does not have a wait_high field
            cfg_if::cfg_if! {
                if #[cfg(not(feature = "esp32"))] {
                    self.register_block().scl_high_period.write(|w| {
                        w.scl_high_period()
                            .bits(scl_high)
                            .scl_wait_high_period()
                            .bits(scl_wait_high)
                    });
                }
                else {
                    self.register_block().scl_high_period.write(|w| {
                        w.scl_high_period()
                            .bits(scl_high)
                    });
                }
            }

            // sda sample
            self.register_block()
                .sda_hold
                .write(|w| w.time().bits(sda_hold));
            self.register_block()
                .sda_sample
                .write(|w| w.time().bits(sda_sample));

            // setup
            self.register_block()
                .scl_rstart_setup
                .write(|w| w.time().bits(setup));
            self.register_block()
                .scl_stop_setup
                .write(|w| w.time().bits(setup));

            // hold
            self.register_block()
                .scl_start_hold
                .write(|w| w.time().bits(hold));
            self.register_block()
                .scl_stop_hold
                .write(|w| w.time().bits(hold));

            // The ESP32 variant does not have an enable flag for the
            // timeout mechanism
            cfg_if::cfg_if! {
                if #[cfg(feature = "esp32")] {
                    // timeout
                    self.register_block()
                        .to
                        .write(|w| w.time_out().bits(tout.into()));
                }
                else {
                    // timeout
                    // FIXME: Enable timout for other chips!
                    self.register_block()
                        .to
                        .write(|w| w.time_out_en().clear_bit());
                }
            }
        }

        Ok(())
    }

    /// Start the actual transmission on a previously configured command set
    ///
    /// This includes the monitoring of the execution in the peripheral and the
    /// return of the operation outcome, including error states
    fn execute_transmission(&mut self) -> Result<(), Error> {
        // Clear all I2C interrupts
        self.register_block()
            .int_clr
            .write(|w| unsafe { w.bits(0x3FFF) });

        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C3 variant)
        #[cfg(feature = "esp32c3")]
        self.register_block()
            .ctr
            .modify(|_, w| w.conf_upgate().set_bit());

        // Start transmission
        self.register_block()
            .ctr
            .modify(|_, w| w.trans_start().set_bit());

        loop {
            let interrupts = self.register_block().int_raw.read();

            // The ESP32 variant has a slightly different interrupt naming
            // scheme!
            cfg_if::cfg_if! {
                if #[cfg(feature = "esp32")] {
                    // Handle error cases
                    if interrupts.time_out_int_raw().bit_is_set() {
                        return Err(Error::TimeOut);
                    } else if interrupts.ack_err_int_raw().bit_is_set() {
                        return Err(Error::AckCheckFailed);
                    } else if interrupts.arbitration_lost_int_raw().bit_is_set() {
                        return Err(Error::ArbitrationLost);
                    }
                }
                else {
                    // Handle error cases
                    if interrupts.time_out_int_raw().bit_is_set() {
                        return Err(Error::TimeOut);
                    } else if interrupts.nack_int_raw().bit_is_set() {
                        return Err(Error::AckCheckFailed);
                    } else if interrupts.arbitration_lost_int_raw().bit_is_set() {
                        return Err(Error::ArbitrationLost);
                    }
                }
            }

            // Handle completion cases
            // A full transmission was completed
            if interrupts.trans_complete_int_raw().bit_is_set()
                || interrupts.end_detect_int_raw().bit_is_set()
            {
                break;
            }
        }

        // Confirm that all commands that were configured were actually executed
        for cmd in self.register_block().comd.iter() {
            if cmd.read().command().bits() != 0x0 && cmd.read().command_done().bit_is_clear() {
                return Err(Error::ExecIncomplete);
            }
        }

        Ok(())
    }

    fn add_write_operation<'a, I>(
        &self,
        addr: u8,
        bytes: &[u8],
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        let cmd_start = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
        // RSTART command
        cmd_start.write(|w| unsafe { w.command().bits(Command::Start.into()) });

        // Load address and R/W bit into FIFO
        self.register_block()
            .data
            .write(|w| unsafe { w.fifo_rdata().bits(addr << 1 | OperationType::WRITE as u8) });
        // Load actual data bytes
        for byte in bytes {
            self.register_block()
                .data
                .write(|w| unsafe { w.fifo_rdata().bits(*byte) });
        }

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        let cmd_write = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
        // WRITE command
        cmd_write.write(|w| unsafe {
            w.command().bits(
                Command::Write {
                    ack_exp: Ack::ACK,
                    ack_check_en: true,
                    length: 1 + bytes.len() as u8,
                }
                .into(),
            )
        });

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        let cmd_stop = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
        // STOP command
        cmd_stop.write(|w| unsafe { w.command().bits(Command::Stop.into()) });

        Ok(())
    }

    fn add_read_operation<'a, I>(
        &self,
        addr: u8,
        buffer: &[u8],
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        cmd_iterator
            .next()
            .ok_or(Error::CommandNrExceeded)?
            .write(|w| unsafe { w.command().bits(Command::Start.into()) });

        // Load address and R/W bit into FIFO
        self.register_block()
            .data
            .write(|w| unsafe { w.fifo_rdata().bits(addr << 1 | OperationType::READ as u8) });

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        cmd_iterator
            .next()
            .ok_or(Error::CommandNrExceeded)?
            .write(|w| unsafe {
                w.command().bits(
                    Command::Write {
                        ack_exp: Ack::ACK,
                        ack_check_en: true,
                        length: 1,
                    }
                    .into(),
                )
            });

        // For reading bytes prior to the last read byte we need to
        // configure the ack
        if buffer.len() > 1 {
            // READ command for first n - 1 bytes
            cmd_iterator
                .next()
                .ok_or(Error::CommandNrExceeded)?
                .write(|w| unsafe {
                    w.command().bits(
                        Command::Read {
                            ack_value: Ack::ACK,
                            length: buffer.len() as u8 - 1,
                        }
                        .into(),
                    )
                });
        }

        // READ command for (last or only) byte
        cmd_iterator
            .next()
            .ok_or(Error::CommandNrExceeded)?
            .write(|w| unsafe {
                w.command().bits(
                    Command::Read {
                        ack_value: Ack::NACK,
                        length: 1,
                    }
                    .into(),
                )
            });

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        cmd_iterator
            .next()
            .ok_or(Error::CommandNrExceeded)?
            .write(|w| unsafe { w.command().bits(Command::Stop.into()) });

        Ok(())
    }

    /// Resets the transmit and receive FIFO buffers
    fn reset_fifo(&mut self) {
        // First, reset the fifo buffers
        self.register_block()
            .fifo_conf
            .modify(|_, w| w.tx_fifo_rst().set_bit().rx_fifo_rst().set_bit());
        self.register_block()
            .fifo_conf
            .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());
    }

    /// Send data bytes from the `bytes` array to a target slave with the
    /// address `addr`
    fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();

        // Split the potentially larger `bytes` array into chunks of (at most) 31
        // entries Together with the addr/access byte at the beginning of every
        // transmission, this is the maximum size that we can store in the
        // (default config) TX FIFO
        for chunk in bytes.chunks(31) {
            // Add write operation
            self.add_write_operation(addr, chunk, &mut self.register_block().comd.iter())?;

            // Start transmission
            self.execute_transmission()?
        }

        Ok(())
    }

    /// Read bytes from a target slave with the address `addr`
    /// The number of read bytes is deterimed by the size of the `buffer`
    /// argument
    fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // If the buffer size is > 32 bytes, this signals the
        // intent to read more than that number of bytes, which we
        // cannot achieve at this point in time
        // TODO: Handle the case where we transfer an amount of data that is exceeding
        // the FIFO size (i.e. > 32 bytes?)
        if buffer.len() > 31 {
            return Err(Error::ExceedingFifo);
        }

        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();

        // Add write operation
        self.add_read_operation(addr, buffer, &mut self.register_block().comd.iter())?;

        // Start transmission
        self.execute_transmission()?;

        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            *byte = self.register_block().data.read().fifo_rdata().bits();
        }

        Ok(())
    }

    /// Write bytes from the `bytes` array first and then read n bytes into
    /// the `buffer` array with n being the size of the array.
    fn master_write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        // If the buffer or bytes size is > 32 bytes, this signals the
        // intent to read/write more than that number of bytes, which we
        // cannot achieve at this point in time
        // TODO: Handle the case where we transfer an amount of data that is exceeding
        // the FIFO size (i.e. > 32 bytes?)
        if buffer.len() > 31 || bytes.len() > 31 {
            return Err(Error::ExceedingFifo);
        }

        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();

        let mut cmd_iterator = self.register_block().comd.iter();

        // Add write operation
        self.add_write_operation(addr, bytes, &mut cmd_iterator)?;

        // Add read operation (which appends commands to the existing command list)
        self.add_read_operation(addr, buffer, &mut cmd_iterator)?;

        // Start transmission
        self.execute_transmission()?;

        // Read bytes from FIFO
        for byte in buffer.iter_mut() {
            *byte = self.register_block().data.read().fifo_rdata().bits();
        }

        Ok(())
    }
}

/// Pins used by the I2C interface
///
/// Note that any two pins may be used
pub struct Pins<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin> {
    pub sda: SDA,
    pub scl: SCL,
}

/// A generic I2C Command
enum Command {
    Start,
    Stop,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode = match c {
            Command::Start => Opcode::RSTART,
            Command::Stop => Opcode::STOP,
            Command::Write { .. } => Opcode::WRITE,
            Command::Read { .. } => Opcode::READ,
        };

        let length = match c {
            Command::Start | Command::Stop => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::Read { .. } => Ack::NACK,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::Write { .. } => Ack::NACK,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        } else {
            cmd &= !(1 << 8);
        }

        if ack_exp == Ack::NACK {
            cmd |= 1 << 9;
        } else {
            cmd &= !(1 << 9);
        }

        if ack_value == Ack::NACK {
            cmd |= 1 << 10;
        } else {
            cmd &= !(1 << 10);
        }

        cmd |= (opcode as u16) << 11;

        cmd
    }
}

enum OperationType {
    WRITE = 0,
    READ  = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    ACK,
    NACK,
}

#[allow(dead_code)]
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
enum Opcode {
    RSTART = 6,
    WRITE  = 1,
    READ   = 3,
    STOP   = 2,
    END    = 4,
}

#[allow(dead_code)]
#[cfg(any(feature = "esp32", feature = "esp32s2"))]
enum Opcode {
    RSTART = 0,
    WRITE  = 1,
    READ   = 3,
    STOP   = 2,
    END    = 4,
}

pub use i2c_peripheral;
