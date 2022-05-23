//! I2C Driver
//!
//! Supports multiple I2C peripheral instances
use core::convert::TryInto;

use embedded_hal::blocking::i2c::*;
use fugit::Hertz;

use crate::{
    gpio::{InputPin, OutputPin},
    pac::i2c0::{RegisterBlock, COMD},
    types::{InputSignal, OutputSignal},
};

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c3")] {
        const SOURCE_CLK_FREQ: Hertz<u32> = Hertz::<u32>::MHz(40);
    } else if #[cfg(feature = "esp32")] {
        const SOURCE_CLK_FREQ: Hertz<u32> = Hertz::<u32>::MHz(80);
    } else if #[cfg(feature = "esp32s2")] {
        const SOURCE_CLK_FREQ: Hertz<u32> = Hertz::<u32>::MHz(80);
    } else {
        const SOURCE_CLK_FREQ: Hertz<u32> = Hertz::<u32>::MHz(40);
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32s2")] {
        const I2C_LL_INTR_MASK: u32 = 0x1ffff;
    } else {
        const I2C_LL_INTR_MASK: u32 = 0x3ffff;
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
            Command::Start => Opcode::RStart,
            Command::Stop => Opcode::Stop,
            Command::Write { .. } => Opcode::Write,
            Command::Read { .. } => Opcode::Read,
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

        cmd |= (opcode as u16) << 11;

        cmd
    }
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

#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
enum Opcode {
    RStart = 6,
    Write  = 1,
    Read   = 3,
    Stop   = 2,
}

#[cfg(any(feature = "esp32", feature = "esp32s2"))]
enum Opcode {
    RStart = 0,
    Write  = 1,
    Read   = 2,
    Stop   = 3,
}

/// I2C peripheral container (I2C)
pub struct I2C<T> {
    peripheral: T,
}

impl<T> Read for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.peripheral.master_read(address, buffer)
    }
}

impl<T> Write for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.peripheral.master_write(addr, bytes)
    }
}

impl<T> WriteRead for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.peripheral.master_write_read(address, bytes, buffer)
    }
}

#[cfg(feature = "esp32")]
type System = crate::pac::DPORT;
#[cfg(not(feature = "esp32"))]
type System = crate::pac::SYSTEM;

impl<T> I2C<T>
where
    T: Instance,
{
    /// Create a new I2C instance
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new<
        SDA: OutputPin<OutputSignal = OutputSignal> + InputPin<InputSignal = InputSignal>,
        SCL: OutputPin<OutputSignal = OutputSignal> + InputPin<InputSignal = InputSignal>,
    >(
        i2c: T,
        mut sda: SDA,
        mut scl: SCL,
        frequency: Hertz<u32>,
        system: &mut System,
    ) -> Result<Self, SetupError> {
        enable_peripheral(&i2c, system);

        let mut i2c = I2C { peripheral: i2c };

        sda.set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(OutputSignal::I2CEXT0_SDA)
            .connect_input_to_peripheral(InputSignal::I2CEXT0_SDA);

        scl.set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(OutputSignal::I2CEXT0_SCL)
            .connect_input_to_peripheral(InputSignal::I2CEXT0_SCL);

        i2c.peripheral.setup(frequency)?;

        Ok(i2c)
    }

    /// Return the raw interface to the underlying peripheral
    pub fn free(self) -> T {
        self.peripheral
    }
}

fn enable_peripheral<T: Instance>(i2c: &T, system: &mut System) {
    // enable peripheral
    #[cfg(feature = "esp32")]
    match i2c.i2c_number() {
        0 => {
            system
                .perip_clk_en
                .modify(|_, w| w.i2c0_ext0_clk_en().set_bit());
            system
                .perip_rst_en
                .modify(|_, w| w.i2c0_ext0_rst().clear_bit());
        }
        1 => {
            system
                .perip_clk_en
                .modify(|_, w| w.i2c_ext1_clk_en().set_bit());
            system
                .perip_rst_en
                .modify(|_, w| w.i2c_ext1_rst().clear_bit());
        }
        _ => panic!(), // will never happen
    }
    #[cfg(not(feature = "esp32"))]
    match i2c.i2c_number() {
        0 => {
            system
                .perip_clk_en0
                .modify(|_, w| w.i2c_ext0_clk_en().set_bit());

            // Take the I2C peripheral out of any pre-existing reset state
            // (shouldn't be the case after a fresh startup, but better be safe)
            system
                .perip_rst_en0
                .modify(|_, w| w.i2c_ext0_rst().clear_bit());
        }
        1 => {
            cfg_if::cfg_if! {
                if #[cfg(not(feature = "esp32c3"))] {
                    system
                    .perip_clk_en0
                    .modify(|_, w| w.i2c_ext1_clk_en().set_bit());

                    // Take the I2C peripheral out of any pre-existing reset state
                    // (shouldn't be the case after a fresh startup, but better be safe)
                    system
                        .perip_rst_en0
                        .modify(|_, w| w.i2c_ext1_rst().clear_bit());
                } else {
                    ()
                }

            }
        }
        _ => unreachable!(), // will never happen
    }
}

/// I2C Peripheral Instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn i2c_number(&self) -> usize;

    fn setup(&mut self, frequency: Hertz<u32>) -> Result<(), SetupError> {
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

        #[cfg(feature = "esp32s2")]
        self.register_block()
            .ctr
            .modify(|_, w| w.ref_always_on().set_bit());

        // Configure filter
        self.set_filter(Some(7), Some(7));

        // Configure frequency
        self.set_frequency(SOURCE_CLK_FREQ, frequency)?;

        // Propagate configuration changes (only necessary with C3 and S3)
        #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
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
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

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
    fn set_frequency(
        &mut self,
        source_clk: Hertz<u32>,
        bus_freq: Hertz<u32>,
    ) -> Result<(), SetupError> {
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32s3", feature = "esp32c3"))] {
                // C3 and S3 have a clock devider mechanism, which we want to configure
                // as high as possible.
                let sclk_div: u8 = (source_clk.raw() / (bus_freq.raw() * 1024) + 1)
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;

                let half_cycle: u16 = ((source_clk.raw() / sclk_div as u32 / bus_freq.raw() / 2) as u32)
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
            } else {
                // For EPS32 and the S2 variant no clock divider mechanism exists.
                let half_cycle: u16 = (source_clk.raw() / (bus_freq.raw() * 2) as u32)
                .try_into()
                .map_err(|_| SetupError::InvalidClkConfig)?;
            }
        }

        // The different chips have highly very different timing configurations, so
        // we're setting these up separately (this might introduce some overhead,
        // but improves readability)
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32c3")] {
                let scl_wait_high: u8 = (if bus_freq.raw() <= 50000 { 0 } else { half_cycle / 8 })
                    .try_into()
                    .map_err(|_| SetupError::InvalidClkConfig)?;
                let scl_high: u16 = half_cycle - scl_wait_high as u16;
                let sda_hold = half_cycle / 4;
                let sda_sample = scl_high / 2;
            } else if #[cfg(feature = "esp32s3")] {
                let scl_high = if bus_freq.raw() <= 50000 { half_cycle } else { half_cycle / 5 * 4 + 4 };
                let scl_wait_high: u8 = (half_cycle - scl_high).try_into().map_err(|_| SetupError::InvalidClkConfig)?;
                let sda_hold = half_cycle / 2;
                let sda_sample = half_cycle / 2;
            } else if #[cfg(feature = "esp32s2")] {
                let scl_high = half_cycle / 2 + 2;
                let scl_wait_high = scl_high - (scl_high/2 +2) + 4; // NOTE the additional +4 compared to ESP-IDF
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2 - 1;
            } else {
                // ESP32 is default (as it is the simplest case and does not even have
                // the wait_high field)
                let scl_high = half_cycle;
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2;
                let tout: u16 = half_cycle * 20;
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
                .modify(|_, w| w.sclk_sel().clear_bit().sclk_div_num().bits(sclk_div - 1));

            // scl period
            self.register_block()
                .scl_low_period
                .write(|w| w.scl_low_period().bits(scl_low - 1));

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
                            .bits(scl_high/2 +2)
                    });
                }
            }

            // we already did that above but on S2 we need this to make it work
            #[cfg(feature = "esp32s2")]
            self.register_block()
                .scl_high_period
                .write(|w| w.scl_wait_high_period().bits(scl_wait_high));

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
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C3 and S3 variant)
        #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
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
        include_stop: bool,
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
        write_fifo(
            self.register_block(),
            addr << 1 | OperationType::Write as u8,
        );
        // Load actual data bytes
        for byte in bytes {
            write_fifo(self.register_block(), *byte);
        }

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        let cmd_write = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
        // WRITE command
        cmd_write.write(|w| unsafe {
            w.command().bits(
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: 1 + bytes.len() as u8,
                }
                .into(),
            )
        });

        if include_stop {
            // Check if we have another cmd register ready, otherwise return appropriate
            // error
            let cmd_stop = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
            // STOP command
            cmd_stop.write(|w| unsafe { w.command().bits(Command::Stop.into()) });
        }

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
        write_fifo(self.register_block(), addr << 1 | OperationType::Read as u8);

        // Check if we have another cmd register ready, otherwise return appropriate
        // error
        cmd_iterator
            .next()
            .ok_or(Error::CommandNrExceeded)?
            .write(|w| unsafe {
                w.command().bits(
                    Command::Write {
                        ack_exp: Ack::Ack,
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
                            ack_value: Ack::Ack,
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
                        ack_value: Ack::Nack,
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
            self.add_write_operation(addr, chunk, &mut self.register_block().comd.iter(), true)?;

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
            *byte = read_fifo(self.register_block());
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
        self.add_write_operation(addr, bytes, &mut cmd_iterator, false)?;

        // Add read operation (which appends commands to the existing command list)
        self.add_read_operation(addr, buffer, &mut cmd_iterator)?;

        // Start transmission
        self.execute_transmission()?;

        // Read bytes from FIFO
        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }
}

#[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data.read().fifo_rdata().bits()
}

#[cfg(not(feature = "esp32"))]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    register_block
        .data
        .write(|w| unsafe { w.fifo_rdata().bits(data) });
}

#[cfg(feature = "esp32s2")]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    let base_addr = register_block.scl_low_period.as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe { (fifo_ptr.read() & 0xff) as u8 }
}

#[cfg(feature = "esp32")]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data.read().fifo_rdata().bits()
}

#[cfg(feature = "esp32")]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    let base_addr = register_block.scl_low_period.as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3FF53000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe {
        fifo_ptr.write_volatile(data as u32);
    }
}

impl Instance for crate::pac::I2C0 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn i2c_number(&self) -> usize {
        0
    }
}

#[cfg(not(feature = "esp32c3"))]
impl Instance for crate::pac::I2C1 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn i2c_number(&self) -> usize {
        1
    }
}
