//! I2C Driver
//!
//! Supports multiple I2C peripheral instances

use core::convert::TryInto;

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{InputPin, OutputPin},
    pac::i2c0::{RegisterBlock, COMD},
    system::PeripheralClockControl,
    types::{InputSignal, OutputSignal},
};

cfg_if::cfg_if! {
    if #[cfg(esp32s2)] {
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

#[cfg(feature = "eh1")]
impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        use embedded_hal_1::i2c::ErrorKind;

        match self {
            Self::ExceedingFifo => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            _ => ErrorKind::Other,
        }
    }
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

#[cfg(any(esp32c2, esp32c3, esp32s3))]
enum Opcode {
    RStart = 6,
    Write  = 1,
    Read   = 3,
    Stop   = 2,
}

#[cfg(any(esp32, esp32s2))]
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

impl<T> embedded_hal::blocking::i2c::Read for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.peripheral.master_read(address, buffer)
    }
}

impl<T> embedded_hal::blocking::i2c::Write for I2C<T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.peripheral.master_write(addr, bytes)
    }
}

impl<T> embedded_hal::blocking::i2c::WriteRead for I2C<T>
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

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::i2c::ErrorType for I2C<T> {
    type Error = Error;
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::i2c::I2c for I2C<T>
where
    T: Instance,
{
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.peripheral.master_read(address, buffer)
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.peripheral.master_write(address, bytes)
    }

    fn write_iter<B>(&mut self, _address: u8, _bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.peripheral.master_write_read(address, bytes, buffer)
    }

    fn write_iter_read<B>(
        &mut self,
        _address: u8,
        _bytes: B,
        _buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }

    fn transaction<'a>(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal_1::i2c::Operation<'a>],
    ) -> Result<(), Self::Error> {
        todo!()
    }

    fn transaction_iter<'a, O>(&mut self, _address: u8, _operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = embedded_hal_1::i2c::Operation<'a>>,
    {
        todo!()
    }
}

impl<T> I2C<T>
where
    T: Instance,
{
    /// Create a new I2C instance
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin>(
        i2c: T,
        mut sda: SDA,
        mut scl: SCL,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Result<Self, SetupError> {
        enable_peripheral(&i2c, peripheral_clock_control);

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

        i2c.peripheral.setup(frequency, clocks)?;

        Ok(i2c)
    }

    /// Return the raw interface to the underlying peripheral
    pub fn free(self) -> T {
        self.peripheral
    }
}

fn enable_peripheral<T: Instance>(i2c: &T, peripheral_clock_control: &mut PeripheralClockControl) {
    // enable peripheral
    match i2c.i2c_number() {
        0 => peripheral_clock_control.enable(crate::system::Peripheral::I2cExt0),
        #[cfg(not(any(esp32c2, esp32c3)))]
        1 => peripheral_clock_control.enable(crate::system::Peripheral::I2cExt1),
        _ => unreachable!(), // will never happen
    }
}

/// I2C Peripheral Instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn i2c_number(&self) -> usize;

    fn setup(&mut self, frequency: HertzU32, clocks: &Clocks) -> Result<(), SetupError> {
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

        #[cfg(esp32s2)]
        self.register_block()
            .ctr
            .modify(|_, w| w.ref_always_on().set_bit());

        // Configure filter
        self.set_filter(Some(7), Some(7));

        // Configure frequency
        self.set_frequency(clocks.i2c_clock.convert(), frequency)?;

        // Propagate configuration changes (only necessary with C2, C3, and S3)
        #[cfg(any(esp32c2, esp32c3, esp32s3))]
        self.register_block()
            .ctr
            .modify(|_, w| w.conf_upgate().set_bit());

        self.reset();

        Ok(())
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    fn reset(&self) {
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
        #[cfg(not(esp32))]
        self.register_block()
            .ctr
            .modify(|_, w| w.fsm_rst().set_bit());
    }

    /// Resets the I2C peripheral's command registers
    fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.register_block().comd.iter() {
            cmd.reset();
        }
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a
    /// pulse must be present to pass the filter
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
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
        source_clk: HertzU32,
        bus_freq: HertzU32,
    ) -> Result<(), SetupError> {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                // C2, C3, and S3 have a clock divider mechanism, which we want to configure
                // as high as possible.
                let sclk_div = source_clk.raw() / (bus_freq.raw() * 1024) + 1;
                let half_cycle = source_clk.raw() / sclk_div as u32 / bus_freq.raw() / 2;
            } else {
                // For EPS32 and the S2 variant no clock divider mechanism exists.
                let half_cycle = source_clk.raw() / bus_freq.raw() / 2;
            }
        }

        // The different chips have highly very different timing configurations, so
        // we're setting these up separately (this might introduce some overhead,
        // but improves readability)
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3))] {
                let scl_wait_high = if bus_freq.raw() <= 50000 { 0 } else { half_cycle / 8 };
                let scl_high = half_cycle - scl_wait_high;
                let sda_hold = half_cycle / 4;
                let sda_sample = scl_high / 2;
            } else if #[cfg(esp32s3)] {
                let scl_high = if bus_freq.raw() <= 50000 { half_cycle } else { half_cycle / 5 * 4 + 4 };
                let scl_wait_high = half_cycle - scl_high;
                let sda_hold = half_cycle / 2;
                let sda_sample = half_cycle / 2;
            } else if #[cfg(esp32s2)] {
                let scl_high = half_cycle / 2 + 2;
                let scl_wait_high = half_cycle - scl_high;
                let sda_hold = half_cycle / 2;
                let sda_sample = half_cycle / 2 - 1;
            } else {
                // ESP32 is default (as it is the simplest case and does not even have
                // the wait_high field)
                let scl_high = half_cycle;
                let sda_hold = half_cycle / 2;
                let sda_sample = scl_high / 2;
                let tout = half_cycle * 20;
            }
        }

        let scl_low = half_cycle;
        let setup = half_cycle - 1;
        let hold = half_cycle - 1;

        #[cfg(not(esp32))]
        let scl_low = scl_low as u16 - 1;

        #[cfg(esp32)]
        let scl_low = scl_low as u16;

        let scl_high = scl_high as u16;

        #[cfg(any(esp32c2, esp32c3, esp32s3))]
        let scl_wait_high = scl_wait_high as u8;

        #[cfg(any(esp32s2))]
        let scl_wait_high = scl_wait_high as u16;

        let sda_hold = sda_hold
            .try_into()
            .map_err(|_| SetupError::InvalidClkConfig)?;

        let setup = setup.try_into().map_err(|_| SetupError::InvalidClkConfig)?;

        let sda_sample = sda_sample
            .try_into()
            .map_err(|_| SetupError::InvalidClkConfig)?;

        let hold = hold.try_into().map_err(|_| SetupError::InvalidClkConfig)?;

        #[cfg(any(esp32c2, esp32c3, esp32s3))]
        let sclk_div = sclk_div as u8;

        unsafe {
            // divider
            #[cfg(any(esp32c2, esp32c3, esp32s3))]
            self.register_block()
                .clk_conf
                .modify(|_, w| w.sclk_sel().clear_bit().sclk_div_num().bits(sclk_div - 1));

            // scl period
            self.register_block()
                .scl_low_period
                .write(|w| w.scl_low_period().bits(scl_low));

            // for high/wait_high we have to differentiate between the chips
            // as the EPS32 does not have a wait_high field
            cfg_if::cfg_if! {
                if #[cfg(not(esp32))] {
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

            // we already did that above but on S2 we need this to make it work
            #[cfg(esp32s2)]
            self.register_block().scl_high_period.write(|w| {
                w.scl_wait_high_period()
                    .bits(scl_wait_high)
                    .scl_high_period()
                    .bits(scl_high)
            });

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
                if #[cfg(esp32)] {
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

    fn perform_write<'a, I>(
        &self,
        addr: u8,
        bytes: &[u8],
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if bytes.len() > 254 {
            // we could support more by adding multiple write operations
            return Err(Error::ExceedingFifo);
        }

        // Clear all I2C interrupts
        self.clear_all_interrupts();

        // RSTART command
        add_cmd(cmd_iterator, Command::Start)?;

        // WRITE command
        add_cmd(
            cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1 + bytes.len() as u8,
            },
        )?;

        add_cmd(cmd_iterator, Command::Stop)?;

        self.update_config();

        // Load address and R/W bit into FIFO
        write_fifo(
            self.register_block(),
            addr << 1 | OperationType::Write as u8,
        );

        let index = self.fill_tx_fifo(bytes);

        self.start_transmission();

        // fill FIFO with remaining bytes
        self.write_remaining_tx_fifo(index, bytes)?;

        self.wait_for_completion()?;

        Ok(())
    }

    fn perform_read<'a, I>(
        &self,
        addr: u8,
        buffer: &mut [u8],
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if buffer.len() > 254 {
            // we could support more by adding multiple read operations
            return Err(Error::ExceedingFifo);
        }

        // Clear all I2C interrupts
        self.clear_all_interrupts();

        // RSTART command
        add_cmd(cmd_iterator, Command::Start)?;

        // WRITE command
        add_cmd(
            cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1,
            },
        )?;

        if buffer.len() > 1 {
            // READ command (N - 1)
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Ack,
                    length: buffer.len() as u8 - 1,
                },
            )?;
        }

        // READ w/o ACK
        add_cmd(
            cmd_iterator,
            Command::Read {
                ack_value: Ack::Nack,
                length: 1,
            },
        )?;

        add_cmd(cmd_iterator, Command::Stop)?;

        self.update_config();

        // Load address and R/W bit into FIFO
        write_fifo(self.register_block(), addr << 1 | OperationType::Read as u8);

        self.start_transmission();

        self.read_all_from_fifo(buffer)?;

        self.wait_for_completion()?;

        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            loop {
                self.check_errors()?;

                let reg = self.register_block().fifo_st.read();
                if reg.rxfifo_raddr().bits() != reg.rxfifo_waddr().bits() {
                    break;
                }
            }

            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }

    #[cfg(any(esp32, esp32s2))]
    fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if buffer.len() > 32 {
            panic!("On ESP32 and ESP32-S2 the max I2C read is limited to 32 bytes");
        }

        // wait for completion - then we can just read the data from FIFO
        // once we change to non-fifo mode to support larger transfers that
        // won't work anymore
        self.wait_for_completion()?;

        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }

    fn clear_all_interrupts(&self) {
        self.register_block()
            .int_clr
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        loop {
            let interrupts = self.register_block().int_raw.read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed
            if interrupts.trans_complete_int_raw().bit_is_set()
                || interrupts.end_detect_int_raw().bit_is_set()
            {
                break;
            }
        }
        for cmd in self.register_block().comd.iter() {
            if cmd.read().command().bits() != 0x0 && cmd.read().command_done().bit_is_clear() {
                return Err(Error::ExecIncomplete);
            }
        }

        Ok(())
    }

    fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.register_block().int_raw.read();

        // The ESP32 variant has a slightly different interrupt naming
        // scheme!
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // Handle error cases
                if interrupts.time_out_int_raw().bit_is_set() {
                    self.reset();
                    return Err(Error::TimeOut);
                } else if interrupts.ack_err_int_raw().bit_is_set() {
                    self.reset();
                    return Err(Error::AckCheckFailed);
                } else if interrupts.arbitration_lost_int_raw().bit_is_set() {
                    self.reset();
                    return Err(Error::ArbitrationLost);
                }
            }
            else {
                // Handle error cases
                if interrupts.time_out_int_raw().bit_is_set() {
                    self.reset();
                    return Err(Error::TimeOut);
                } else if interrupts.nack_int_raw().bit_is_set() {
                    self.reset();
                    return Err(Error::AckCheckFailed);
                } else if interrupts.arbitration_lost_int_raw().bit_is_set() {
                    return Err(Error::ArbitrationLost);
                }
            }
        }

        Ok(())
    }

    fn update_config(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C3 and S3 variant)
        #[cfg(any(esp32c2, esp32c3, esp32s3))]
        self.register_block()
            .ctr
            .modify(|_, w| w.conf_upgate().set_bit());
    }

    fn start_transmission(&self) {
        // Start transmission
        self.register_block()
            .ctr
            .modify(|_, w| w.trans_start().set_bit());
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        let mut index = 0;
        while index < bytes.len()
            && !self
                .register_block()
                .int_raw
                .read()
                .txfifo_ovf_int_raw()
                .bit_is_set()
        {
            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
        if self
            .register_block()
            .int_raw
            .read()
            .txfifo_ovf_int_raw()
            .bit_is_set()
        {
            index -= 1;
            self.register_block()
                .int_clr
                .write(|w| w.txfifo_ovf_int_clr().set_bit());
        }
        index
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            while !self
                .register_block()
                .int_raw
                .read()
                .txfifo_wm_int_raw()
                .bit_is_set()
            {}
            self.register_block()
                .int_clr
                .write(|w| w.txfifo_wm_int_clr().set_bit());

            if index >= bytes.len() {
                break Ok(());
            }

            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
    }

    #[cfg(any(esp32, esp32s2))]
    fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if bytes.len() > 31 {
            panic!("On ESP32 and ESP32-S2 the max I2C transfer is limited to 31 bytes");
        }

        for b in bytes {
            write_fifo(self.register_block(), *b);
        }

        bytes.len()
    }

    #[cfg(any(esp32, esp32s2))]
    fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if start_index >= bytes.len() {
            return Ok(());
        }

        // this is only possible when writing the I2C address in release mode
        // from [perform_write_read]
        for b in bytes {
            write_fifo(self.register_block(), *b);
            self.check_errors()?;
        }

        Ok(())
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(not(esp32))]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.register_block().fifo_conf.modify(|_, w| {
            w.tx_fifo_rst()
                .set_bit()
                .rx_fifo_rst()
                .set_bit()
                .nonfifo_en()
                .clear_bit()
                .fifo_prt_en()
                .set_bit()
                .rxfifo_wm_thrhd()
                .variant(1)
                .txfifo_wm_thrhd()
                .variant(8)
        });

        self.register_block()
            .fifo_conf
            .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());

        self.register_block().int_clr.write(|w| {
            w.rxfifo_wm_int_clr()
                .set_bit()
                .txfifo_wm_int_clr()
                .set_bit()
        });

        self.update_config();
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(esp32)]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.register_block().fifo_conf.modify(|_, w| {
            w.tx_fifo_rst()
                .set_bit()
                .rx_fifo_rst()
                .set_bit()
                .nonfifo_en()
                .clear_bit()
                .nonfifo_rx_thres()
                .variant(1)
                .nonfifo_tx_thres()
                .variant(32)
        });

        self.register_block()
            .fifo_conf
            .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.rxfifo_full_int_clr().set_bit());
    }

    /// Send data bytes from the `bytes` array to a target slave with the
    /// address `addr`
    fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        self.perform_write(addr, bytes, &mut self.register_block().comd.iter())?;
        Ok(())
    }

    /// Read bytes from a target slave with the address `addr`
    /// The number of read bytes is deterimed by the size of the `buffer`
    /// argument
    fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        self.perform_read(addr, buffer, &mut self.register_block().comd.iter())?;
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
        // it would be possible to combine the write and read
        // in one transaction but filling the tx fifo with
        // the current code is somewhat slow even in release mode
        // which can cause issues
        self.master_write(addr, bytes)?;
        self.master_read(addr, buffer)?;
        Ok(())
    }
}

fn add_cmd<'a, I>(cmd_iterator: &mut I, command: Command) -> Result<(), Error>
where
    I: Iterator<Item = &'a COMD>,
{
    let cmd = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
    cmd.write(|w| unsafe { w.command().bits(command.into()) });
    Ok(())
}

#[cfg(not(any(esp32, esp32s2)))]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data.read().fifo_rdata().bits()
}

#[cfg(not(esp32))]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    register_block
        .data
        .write(|w| unsafe { w.fifo_rdata().bits(data) });
}

#[cfg(esp32s2)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    let base_addr = register_block.scl_low_period.as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe { (fifo_ptr.read() & 0xff) as u8 }
}

#[cfg(esp32)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data.read().fifo_rdata().bits()
}

#[cfg(esp32)]
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

#[cfg(not(any(esp32c2, esp32c3)))]
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
