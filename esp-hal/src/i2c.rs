//! # I2C Driver
//!
//! ## Overview
//! The I2C Peripheral Driver for ESP chips is a software module that
//! facilitates communication with I2C devices using ESP microcontroller chips.
//! It provides an interface to initialize, configure, and perform read and
//! write operations over the I2C bus.
//!
//! The driver supports features such as handling transmission errors,
//! asynchronous operations, and interrupt-based communication, also supports
//! multiple I2C peripheral instances on `ESP32`, `ESP32H2`, `ESP32S2`, and
//! `ESP32S3` chips
//!
//! ## Example
//! Following code shows how to read data from a BMP180 sensor using I2C.
//!
//! ```no_run
//! // Create a new peripheral object with the described wiring
//! // and standard I2C clock speed
//! let mut i2c = I2C::new(
//!     peripherals.I2C0,
//!     io.pins.gpio1,
//!     io.pins.gpio2,
//!     100.kHz(),
//!     &clocks,
//! );
//! loop {
//!     let mut data = [0u8; 22];
//!     i2c.write_read(0x77, &[0xaa], &mut data).ok();
//!
//!     println!("{:02x?}", data);
//! }
//! ```

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::i2c0::{RegisterBlock, COMD},
    system::PeripheralClockControl,
};

cfg_if::cfg_if! {
    if #[cfg(esp32s2)] {
        const I2C_LL_INTR_MASK: u32 = 0x1ffff;
    } else {
        const I2C_LL_INTR_MASK: u32 = 0x3ffff;
    }
}

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    ExceedingFifo,
    AckCheckFailed,
    TimeOut,
    ArbitrationLost,
    ExecIncomplete,
    CommandNrExceeded,
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::ErrorKind;

        match self {
            Self::ExceedingFifo => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            _ => ErrorKind::Other,
        }
    }
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

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
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
pub struct I2C<'d, T> {
    peripheral: PeripheralRef<'d, T>,
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::i2c::Read for I2C<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.peripheral.master_read(address, buffer)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::i2c::Write for I2C<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.peripheral.master_write(addr, bytes)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::i2c::WriteRead for I2C<'_, T>
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

#[cfg(feature = "embedded-hal")]
impl<T> embedded_hal::i2c::ErrorType for I2C<'_, T> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<T> embedded_hal::i2c::I2c for I2C<'_, T>
where
    T: Instance,
{
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.peripheral.master_read(address, buffer)
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.peripheral.master_write(address, bytes)
    }

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.peripheral.master_write_read(address, bytes, buffer)
    }

    fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<'d, T> I2C<'d, T>
where
    T: Instance,
{
    /// Create a new I2C instance
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin>(
        i2c: impl Peripheral<P = T> + 'd,
        sda: impl Peripheral<P = SDA> + 'd,
        scl: impl Peripheral<P = SCL> + 'd,
        frequency: HertzU32,
        clocks: &Clocks,
    ) -> Self {
        Self::new_with_timeout(i2c, sda, scl, frequency, clocks, None)
    }

    /// Create a new I2C instance with a custom timeout value.
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new_with_timeout<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin>(
        i2c: impl Peripheral<P = T> + 'd,
        sda: impl Peripheral<P = SDA> + 'd,
        scl: impl Peripheral<P = SCL> + 'd,
        frequency: HertzU32,
        clocks: &Clocks,
        timeout: Option<u32>,
    ) -> Self {
        crate::into_ref!(i2c, sda, scl);

        PeripheralClockControl::enable(match i2c.i2c_number() {
            0 => crate::system::Peripheral::I2cExt0,
            #[cfg(i2c1)]
            1 => crate::system::Peripheral::I2cExt1,
            _ => unreachable!(), // will never happen
        });

        let mut i2c = I2C { peripheral: i2c };

        // avoid SCL/SDA going low during configuration
        scl.set_output_high(true);
        sda.set_output_high(true);

        scl.set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(i2c.peripheral.scl_output_signal())
            .connect_input_to_peripheral(i2c.peripheral.scl_input_signal());

        sda.set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(i2c.peripheral.sda_output_signal())
            .connect_input_to_peripheral(i2c.peripheral.sda_input_signal());

        i2c.peripheral.setup(frequency, clocks, timeout);

        i2c
    }

    #[cfg(feature = "async")]
    pub(crate) fn inner(&self) -> &T {
        &self.peripheral
    }
}

#[cfg(feature = "async")]
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use cfg_if::cfg_if;
    use embassy_futures::select::select;
    use embassy_sync::waitqueue::AtomicWaker;
    use embedded_hal::i2c::Operation;
    use procmacros::interrupt;

    use super::*;

    cfg_if! {
        if #[cfg(all(i2c0, i2c1))] {
            const NUM_I2C: usize = 2;
        } else if #[cfg(i2c0)] {
            const NUM_I2C: usize = 1;
        }
    }

    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKERS: [AtomicWaker; NUM_I2C] = [INIT; NUM_I2C];

    pub(crate) enum Event {
        EndDetect,
        TxComplete,
        #[cfg(not(any(esp32, esp32s2)))]
        TxFifoWatermark,
    }

    pub(crate) struct I2cFuture<'a, T>
    where
        T: Instance,
    {
        event: Event,
        instance: &'a T,
    }

    impl<'a, T> I2cFuture<'a, T>
    where
        T: Instance,
    {
        pub fn new(event: Event, instance: &'a T) -> Self {
            instance
                .register_block()
                .int_ena()
                .modify(|_, w| match event {
                    Event::EndDetect => w.end_detect().set_bit(),
                    Event::TxComplete => w.trans_complete().set_bit(),
                    #[cfg(not(any(esp32, esp32s2)))]
                    Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
                });

            Self { event, instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            let r = self.instance.register_block().int_ena().read();

            match self.event {
                Event::EndDetect => r.end_detect().bit_is_clear(),
                Event::TxComplete => r.trans_complete().bit_is_clear(),
                #[cfg(not(any(esp32, esp32s2)))]
                Event::TxFifoWatermark => r.txfifo_wm().bit_is_clear(),
            }
        }
    }

    impl<'a, T> core::future::Future for I2cFuture<'a, T>
    where
        T: Instance,
    {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKERS[self.instance.i2c_number()].register(ctx.waker());

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<T> I2C<'_, T>
    where
        T: Instance,
    {
        async fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
            // Reset FIFO and command list
            self.peripheral.reset_fifo();
            self.peripheral.reset_command_list();

            self.perform_read(
                addr,
                buffer,
                &mut self.peripheral.register_block().comd_iter(),
            )
            .await
        }

        async fn perform_read<'a, I>(
            &self,
            addr: u8,
            buffer: &mut [u8],
            cmd_iterator: &mut I,
        ) -> Result<(), Error>
        where
            I: Iterator<Item = &'a COMD>,
        {
            self.peripheral.setup_read(addr, buffer, cmd_iterator)?;
            self.peripheral.start_transmission();

            self.read_all_from_fifo(buffer).await?;
            self.wait_for_completion().await?;

            Ok(())
        }

        #[cfg(any(esp32, esp32s2))]
        async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
            if buffer.len() > 32 {
                panic!("On ESP32 and ESP32-S2 the max I2C read is limited to 32 bytes");
            }

            self.wait_for_completion().await?;

            for byte in buffer.iter_mut() {
                *byte = read_fifo(self.peripheral.register_block());
            }

            Ok(())
        }

        #[cfg(not(any(esp32, esp32s2)))]
        async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
            self.peripheral.read_all_from_fifo(buffer)
        }

        async fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
            // Reset FIFO and command list
            self.peripheral.reset_fifo();
            self.peripheral.reset_command_list();

            self.perform_write(
                addr,
                bytes,
                &mut self.peripheral.register_block().comd_iter(),
            )
            .await
        }

        async fn perform_write<'a, I>(
            &self,
            addr: u8,
            bytes: &[u8],
            cmd_iterator: &mut I,
        ) -> Result<(), Error>
        where
            I: Iterator<Item = &'a COMD>,
        {
            self.peripheral.setup_write(addr, bytes, cmd_iterator)?;
            let index = self.peripheral.fill_tx_fifo(bytes);
            self.peripheral.start_transmission();

            // Fill the FIFO with the remaining bytes:
            self.write_remaining_tx_fifo(index, bytes).await?;
            self.wait_for_completion().await?;

            Ok(())
        }

        #[cfg(any(esp32, esp32s2))]
        async fn write_remaining_tx_fifo(
            &self,
            start_index: usize,
            bytes: &[u8],
        ) -> Result<(), Error> {
            if start_index >= bytes.len() {
                return Ok(());
            }

            for b in bytes {
                write_fifo(self.peripheral.register_block(), *b);
                self.peripheral.check_errors()?;
            }

            Ok(())
        }

        #[cfg(not(any(esp32, esp32s2)))]
        async fn write_remaining_tx_fifo(
            &self,
            start_index: usize,
            bytes: &[u8],
        ) -> Result<(), Error> {
            let mut index = start_index;
            loop {
                self.peripheral.check_errors()?;

                I2cFuture::new(Event::TxFifoWatermark, self.inner()).await;

                self.peripheral
                    .register_block()
                    .int_clr()
                    .write(|w| w.txfifo_wm().clear_bit_by_one());

                I2cFuture::new(Event::TxFifoWatermark, self.inner()).await;

                if index >= bytes.len() {
                    break Ok(());
                }

                write_fifo(self.peripheral.register_block(), bytes[index]);
                index += 1;
            }
        }

        async fn wait_for_completion(&self) -> Result<(), Error> {
            self.peripheral.check_errors()?;

            select(
                I2cFuture::new(Event::TxComplete, self.inner()),
                I2cFuture::new(Event::EndDetect, self.inner()),
            )
            .await;

            for cmd in self.peripheral.register_block().comd_iter() {
                if cmd.read().command().bits() != 0x0 && cmd.read().command_done().bit_is_clear() {
                    return Err(Error::ExecIncomplete);
                }
            }

            Ok(())
        }
    }

    impl<'d, T> embedded_hal_async::i2c::I2c for I2C<'d, T>
    where
        T: Instance,
    {
        async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            self.master_read(address, read).await
        }

        async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
            self.master_write(address, write).await
        }

        async fn write_read(
            &mut self,
            address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.master_write(address, write).await?;
            self.master_read(address, read).await?;

            Ok(())
        }

        async fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            todo!()
        }
    }

    #[interrupt]
    fn I2C_EXT0() {
        unsafe { &*crate::peripherals::I2C0::PTR }
            .int_ena()
            .modify(|_, w| w.end_detect().clear_bit().trans_complete().clear_bit());

        #[cfg(not(any(esp32, esp32s2)))]
        unsafe { &*crate::peripherals::I2C0::PTR }
            .int_ena()
            .modify(|_, w| w.txfifo_wm().clear_bit());

        WAKERS[0].wake();
    }

    #[cfg(i2c1)]
    #[interrupt]
    fn I2C_EXT1() {
        unsafe { &*crate::peripherals::I2C1::PTR }
            .int_ena()
            .modify(|_, w| w.end_detect().clear_bit().trans_complete().clear_bit());

        #[cfg(not(any(esp32, esp32s2)))]
        unsafe { &*crate::peripherals::I2C0::PTR }
            .int_ena()
            .modify(|_, w| w.txfifo_wm().clear_bit());

        WAKERS[1].wake();
    }
}

/// I2C Peripheral Instance
pub trait Instance: crate::private::Sealed {
    fn scl_output_signal(&self) -> OutputSignal;
    fn scl_input_signal(&self) -> InputSignal;
    fn sda_output_signal(&self) -> OutputSignal;
    fn sda_input_signal(&self) -> InputSignal;

    fn register_block(&self) -> &RegisterBlock;

    fn i2c_number(&self) -> usize;

    fn setup(&mut self, frequency: HertzU32, clocks: &Clocks, timeout: Option<u32>) {
        self.register_block().ctr().modify(|_, w| unsafe {
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
            .ctr()
            .modify(|_, w| w.ref_always_on().set_bit());

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        self.set_filter(Some(7), Some(7));

        // Configure frequency
        #[cfg(esp32)]
        self.set_frequency(clocks.i2c_clock.convert(), frequency, timeout);
        #[cfg(esp32s2)]
        self.set_frequency(clocks.apb_clock.convert(), frequency, timeout);
        #[cfg(not(any(esp32, esp32s2)))]
        self.set_frequency(clocks.xtal_clock.convert(), frequency, timeout);

        self.update_config();

        // Reset entire peripheral (also resets fifo)
        self.reset();
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    fn reset(&self) {
        // Reset the FSM
        // (the option to reset the FSM is not available
        // for the ESP32)
        #[cfg(not(esp32))]
        self.register_block()
            .ctr()
            .modify(|_, w| w.fsm_rst().set_bit());

        // Clear all I2C interrupts
        self.register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Resets the I2C peripheral's command registers
    fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.register_block().comd_iter() {
            cmd.reset();
        }
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a
    /// pulse must be present to pass the filter
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let sda_register = &self.register_block().sda_filter_cfg();
                let scl_register = &self.register_block().scl_filter_cfg();
            } else {
                let sda_register = &self.register_block().filter_cfg();
                let scl_register = &self.register_block().filter_cfg();
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

    #[cfg(esp32)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&mut self, source_clk: HertzU32, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        let tout = if let Some(timeout) = timeout {
            timeout
        } else {
            // default we set the timeout value to 10 bus cycles
            half_cycle * 20
        };

        // SCL period. According to the TRM, we should always subtract 1 to SCL low
        // period
        let scl_low = scl_low - 1;
        // Still according to the TRM, if filter is not enbled, we have to subtract 7,
        // if SCL filter is enabled, we have to subtract:
        //   8 if SCL filter is between 0 and 2 (included)
        //   6 + SCL threshold if SCL filter is between 3 and 7 (included)
        // to SCL high period
        let mut scl_high = scl_high;
        // In the "worst" case, we will subtract 13, make sure the result will still be
        // correct

        // FIXME since we always set the filter threshold to 7 we don't need conditional
        // code here once that changes we need the conditional code here
        scl_high -= 7 + 6;

        // if (filter_cfg_en) {
        //     if (thres <= 2) {
        //         scl_high -= 8;
        //     } else {
        //         assert(hw->scl_filter_cfg.thres <= 7);
        //         scl_high -= thres + 6;
        //     }
        // } else {
        //    scl_high -= 7;
        //}

        let scl_high_period = scl_high;
        let scl_low_period = scl_low;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold;
        let scl_stop_hold_time = hold;
        let time_out_value = tout;

        self.configure_clock(
            0,
            scl_low_period,
            scl_high_period,
            0,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            time_out_value,
            true,
        );
    }

    #[cfg(esp32s2)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&mut self, source_clk: HertzU32, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        let scl_high = half_cycle / 2 + 2;
        let scl_wait_high = half_cycle - scl_high;
        let sda_hold = half_cycle / 2;
        // scl_wait_high < sda_sample <= scl_high
        let sda_sample = half_cycle / 2 - 1;
        let setup = half_cycle;
        let hold = half_cycle;
        let tout = if let Some(timeout) = timeout {
            timeout
        } else {
            // default we set the timeout value to 10 bus cycles
            half_cycle * 20
        };

        // scl period
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold;
        let time_out_value = tout;
        let time_out_en = true;

        self.configure_clock(
            0,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            time_out_value,
            time_out_en,
        );
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&mut self, source_clk: HertzU32, bus_freq: HertzU32, timeout: Option<u32>) {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

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
        let sda_sample = half_cycle / 2 + scl_wait_high;
        let setup = half_cycle;
        let hold = half_cycle;

        let tout = if let Some(timeout) = timeout {
            timeout
        } else {
            // default we set the timeout value to about 10 bus cycles
            // log(20*half_cycle)/log(2) = log(half_cycle)/log(2) +  log(20)/log(2)
            (4 * 8 - (5 * half_cycle).leading_zeros()) + 2
        };

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

        self.configure_clock(
            clkm_div,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            time_out_value,
            time_out_en,
        );
    }

    #[allow(clippy::too_many_arguments, unused)]
    fn configure_clock(
        &mut self,
        sclk_div: u32,
        scl_low_period: u32,
        scl_high_period: u32,
        scl_wait_high_period: u32,
        sda_hold_time: u32,
        sda_sample_time: u32,
        scl_rstart_setup_time: u32,
        scl_stop_setup_time: u32,
        scl_start_hold_time: u32,
        scl_stop_hold_time: u32,
        time_out_value: u32,
        time_out_en: bool,
    ) {
        unsafe {
            // divider
            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            self.register_block().clk_conf().modify(|_, w| {
                w.sclk_sel()
                    .clear_bit()
                    .sclk_div_num()
                    .bits((sclk_div - 1) as u8)
            });

            // scl period
            self.register_block()
                .scl_low_period()
                .write(|w| w.scl_low_period().bits(scl_low_period as u16));

            // for high/wait_high we have to differentiate between the chips
            // as the EPS32 does not have a wait_high field
            cfg_if::cfg_if! {
                if #[cfg(not(esp32))] {
                    self.register_block().scl_high_period().write(|w| {
                        w.scl_high_period()
                            .bits(scl_high_period as u16)
                            .scl_wait_high_period()
                            .bits(scl_wait_high_period.try_into().unwrap())
                    });
                }
                else {
                    self.register_block().scl_high_period().write(|w| {
                        w.scl_high_period()
                            .bits(scl_high_period as u16)
                    });
                }
            }

            // we already did that above but on S2 we need this to make it work
            #[cfg(esp32s2)]
            self.register_block().scl_high_period().write(|w| {
                w.scl_wait_high_period()
                    .bits(scl_wait_high_period as u16)
                    .scl_high_period()
                    .bits(scl_high_period as u16)
            });

            // sda sample
            self.register_block()
                .sda_hold()
                .write(|w| w.time().bits(sda_hold_time as u16));
            self.register_block()
                .sda_sample()
                .write(|w| w.time().bits(sda_sample_time as u16));

            // setup
            self.register_block()
                .scl_rstart_setup()
                .write(|w| w.time().bits(scl_rstart_setup_time as u16));
            self.register_block()
                .scl_stop_setup()
                .write(|w| w.time().bits(scl_stop_setup_time as u16));

            // hold
            self.register_block()
                .scl_start_hold()
                .write(|w| w.time().bits(scl_start_hold_time as u16));
            self.register_block()
                .scl_stop_hold()
                .write(|w| w.time().bits(scl_stop_hold_time as u16));

            // The ESP32 variant does not have an enable flag for the
            // timeout mechanism
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    // timeout
                    self.register_block()
                        .to()
                        .write(|w| w.time_out().bits(time_out_value));
                }
                else {
                    // timeout
                    // FIXME: Enable timout for other chips!
                    #[allow(clippy::useless_conversion)]
                    self.register_block()
                        .to()
                        .write(|w| w.time_out_en().bit(time_out_en)
                        .time_out_value()
                        .variant(time_out_value.try_into().unwrap())
                    );
                }
            }
        }
    }

    fn setup_write<'a, I>(&self, addr: u8, bytes: &[u8], cmd_iterator: &mut I) -> Result<(), Error>
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
        self.setup_write(addr, bytes, cmd_iterator)?;
        let index = self.fill_tx_fifo(bytes);
        self.start_transmission();

        // Fill the FIFO with the remaining bytes:
        self.write_remaining_tx_fifo(index, bytes)?;
        self.wait_for_completion()?;

        Ok(())
    }

    fn setup_read<'a, I>(
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
        self.setup_read(addr, buffer, cmd_iterator)?;
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

                let reg = self.register_block().fifo_st().read();
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
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        loop {
            let interrupts = self.register_block().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed
            if interrupts.trans_complete().bit_is_set() || interrupts.end_detect().bit_is_set() {
                break;
            }
        }
        for cmd in self.register_block().comd_iter() {
            if cmd.read().command().bits() != 0x0 && cmd.read().command_done().bit_is_clear() {
                return Err(Error::ExecIncomplete);
            }
        }

        Ok(())
    }

    fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.register_block().int_raw().read();

        // The ESP32 variant has a slightly different interrupt naming
        // scheme!
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // Handle error cases
                if interrupts.time_out().bit_is_set() {
                    self.reset();
                    return Err(Error::TimeOut);
                } else if interrupts.ack_err().bit_is_set() {
                    self.reset();
                    return Err(Error::AckCheckFailed);
                } else if interrupts.arbitration_lost().bit_is_set() {
                    self.reset();
                    return Err(Error::ArbitrationLost);
                }
            }
            else {
                // Handle error cases
                if interrupts.time_out().bit_is_set() {
                    self.reset();
                    return Err(Error::TimeOut);
                } else if interrupts.nack().bit_is_set() {
                    self.reset();
                    return Err(Error::AckCheckFailed);
                } else if interrupts.arbitration_lost().bit_is_set() {
                    self.reset();
                    return Err(Error::ArbitrationLost);
                } else if  interrupts.trans_complete().bit_is_set() && self.register_block().sr().read().resp_rec().bit_is_clear() {
                    self.reset();
                    return Err(Error::AckCheckFailed);
                }
            }
        }

        Ok(())
    }

    fn update_config(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C2, C3, C6, H2 and S3 variant)
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        self.register_block()
            .ctr()
            .modify(|_, w| w.conf_upgate().set_bit());
    }

    fn start_transmission(&self) {
        // Start transmission
        self.register_block()
            .ctr()
            .modify(|_, w| w.trans_start().set_bit());
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        let mut index = 0;
        while index < bytes.len()
            && !self
                .register_block()
                .int_raw()
                .read()
                .txfifo_ovf()
                .bit_is_set()
        {
            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
        if self
            .register_block()
            .int_raw()
            .read()
            .txfifo_ovf()
            .bit_is_set()
        {
            index -= 1;
            self.register_block()
                .int_clr()
                .write(|w| w.txfifo_ovf().clear_bit_by_one());
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
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {}

            self.register_block()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            while !self
                .register_block()
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {}

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
        self.register_block().fifo_conf().modify(|_, w| {
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
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());

        self.register_block().int_clr().write(|w| {
            w.rxfifo_wm()
                .clear_bit_by_one()
                .txfifo_wm()
                .clear_bit_by_one()
        });

        self.update_config();
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(esp32)]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.register_block().fifo_conf().modify(|_, w| {
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
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());

        self.register_block()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    /// Send data bytes from the `bytes` array to a target slave with the
    /// address `addr`
    fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        self.perform_write(addr, bytes, &mut self.register_block().comd_iter())?;
        Ok(())
    }

    /// Read bytes from a target slave with the address `addr`
    /// The number of read bytes is deterimed by the size of the `buffer`
    /// argument
    fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        self.perform_read(addr, buffer, &mut self.register_block().comd_iter())?;
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
    register_block.data().read().fifo_rdata().bits()
}

#[cfg(not(esp32))]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    register_block
        .data()
        .write(|w| unsafe { w.fifo_rdata().bits(data) });
}

#[cfg(esp32s2)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
}

#[cfg(esp32)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data().read().fifo_rdata().bits()
}

#[cfg(esp32)]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3FF53000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe {
        fifo_ptr.write_volatile(data as u32);
    }
}

impl Instance for crate::peripherals::I2C0 {
    #[inline(always)]
    fn scl_output_signal(&self) -> OutputSignal {
        OutputSignal::I2CEXT0_SCL
    }

    #[inline(always)]
    fn scl_input_signal(&self) -> InputSignal {
        InputSignal::I2CEXT0_SCL
    }

    #[inline(always)]
    fn sda_output_signal(&self) -> OutputSignal {
        OutputSignal::I2CEXT0_SDA
    }

    fn sda_input_signal(&self) -> InputSignal {
        InputSignal::I2CEXT0_SDA
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn i2c_number(&self) -> usize {
        0
    }
}

#[cfg(i2c1)]
impl Instance for crate::peripherals::I2C1 {
    #[inline(always)]
    fn scl_output_signal(&self) -> OutputSignal {
        OutputSignal::I2CEXT1_SCL
    }

    #[inline(always)]
    fn scl_input_signal(&self) -> InputSignal {
        InputSignal::I2CEXT1_SCL
    }

    #[inline(always)]
    fn sda_output_signal(&self) -> OutputSignal {
        OutputSignal::I2CEXT1_SDA
    }

    fn sda_input_signal(&self) -> InputSignal {
        InputSignal::I2CEXT1_SDA
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn i2c_number(&self) -> usize {
        1
    }
}

#[cfg(lp_i2c0)]
pub mod lp_i2c {
    //! Low-power I2C driver

    use fugit::HertzU32;

    use crate::{
        gpio::{lp_gpio::LowPowerPin, OpenDrain},
        peripherals::{LP_CLKRST, LP_I2C0},
    };

    const LP_I2C_FILTER_CYC_NUM_DEF: u8 = 7;

    /// I2C-specific transmission errors
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Error {
        ExceedingFifo,
        AckCheckFailed,
        TimeOut,
        ArbitrationLost,
        ExecIncomplete,
        CommandNrExceeded,
        InvalidResponse,
    }

    #[allow(unused)]
    enum OperationType {
        Write = 0,
        Read  = 1,
    }

    #[allow(unused)]
    #[derive(Eq, PartialEq, Copy, Clone)]
    enum Ack {
        Ack,
        Nack,
    }

    #[allow(unused)]
    enum Opcode {
        RStart = 6,
        Write  = 1,
        Read   = 3,
        Stop   = 2,
        End    = 4,
    }

    #[derive(PartialEq)]
    #[allow(unused)]
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
            let opcode = match c {
                Command::Start => Opcode::RStart,
                Command::Stop => Opcode::Stop,
                Command::End => Opcode::End,
                Command::Write { .. } => Opcode::Write,
                Command::Read { .. } => Opcode::Read,
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

    impl From<Command> for u32 {
        fn from(c: Command) -> u32 {
            let opcode = match c {
                Command::Start => Opcode::RStart,
                Command::Stop => Opcode::Stop,
                Command::End => Opcode::End,
                Command::Write { .. } => Opcode::Write,
                Command::Read { .. } => Opcode::Read,
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

            let mut cmd: u32 = length.into();

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

            cmd |= (opcode as u32) << 11;

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

    pub struct LpI2c {
        i2c: LP_I2C0,
    }

    impl LpI2c {
        pub fn new(
            i2c: LP_I2C0,
            _sda: LowPowerPin<OpenDrain, 6>,
            _scl: LowPowerPin<OpenDrain, 7>,
            frequency: HertzU32,
        ) -> Self {
            let me = Self { i2c };

            // Configure LP I2C GPIOs
            // Initialize IO Pins
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };
            let lp_peri = unsafe { &*crate::peripherals::LP_PERI::PTR };

            unsafe {
                lp_aon
                    .gpio_mux()
                    .modify(|r, w| w.sel().bits(r.sel().bits() | (1 << 6)));
                lp_aon
                    .gpio_mux()
                    .modify(|r, w| w.sel().bits(r.sel().bits() | (1 << 7)));
                lp_io.gpio6().modify(|_, w| w.mcu_sel().variant(1)); // TODO

                lp_io.gpio7().modify(|_, w| w.mcu_sel().variant(1));

                // Set output mode to Normal
                lp_io.pin6().modify(|_, w| w.pad_driver().set_bit());
                // Enable output (writing to write-1-to-set register, then internally the
                // `GPIO_OUT_REG` will be set)
                lp_io
                    .out_enable_w1ts()
                    .write(|w| w.enable_w1ts().bits(1 << 6));
                // Enable input
                lp_io.gpio6().modify(|_, w| w.fun_ie().set_bit());

                // Disable pulldown (enable internal weak pull-down)
                lp_io.gpio6().modify(|_, w| w.fun_wpd().clear_bit());
                // Enable pullup
                lp_io.gpio6().modify(|_, w| w.fun_wpu().set_bit());

                // Same process for SCL pin
                lp_io.pin7().modify(|_, w| w.pad_driver().set_bit());
                // Enable output (writing to write-1-to-set register, then internally the
                // `GPIO_OUT_REG` will be set)
                lp_io
                    .out_enable_w1ts()
                    .write(|w| w.enable_w1ts().bits(1 << 7));
                // Enable input
                lp_io.gpio7().modify(|_, w| w.fun_ie().set_bit());
                // Disable pulldown (enable internal weak pull-down)
                lp_io.gpio7().modify(|_, w| w.fun_wpd().clear_bit());
                // Enable pullup
                lp_io.gpio7().modify(|_, w| w.fun_wpu().set_bit());

                // Select LP I2C function for the SDA and SCL pins
                lp_io.gpio6().modify(|_, w| w.mcu_sel().bits(1));
                lp_io.gpio7().modify(|_, w| w.mcu_sel().bits(1));
            }

            // Initialize LP I2C HAL */
            me.i2c.clk_conf().modify(|_, w| w.sclk_active().set_bit());

            // Enable LP I2C controller clock
            lp_peri
                .clk_en()
                .modify(|_, w| w.lp_ext_i2c_ck_en().set_bit());

            lp_peri
                .reset_en()
                .modify(|_, w| w.lp_ext_i2c_reset_en().set_bit());
            lp_peri
                .reset_en()
                .modify(|_, w| w.lp_ext_i2c_reset_en().clear_bit());

            // Set LP I2C source clock
            unsafe { &*LP_CLKRST::PTR }
                .lpperi()
                .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

            // Initialize LP I2C Master mode
            me.i2c.ctr().modify(|_, w| unsafe {
                // Clear register
                w.bits(0)
                    // Use open drain output for SDA and SCL
                    .sda_force_out()
                    .set_bit()
                    .scl_force_out()
                    .set_bit()
                    // Ensure that clock is enabled
                    .clk_en()
                    .set_bit()
            });

            // First, reset the fifo buffers
            me.i2c.fifo_conf().modify(|_, w| w.nonfifo_en().clear_bit());

            me.i2c
                .ctr()
                .modify(|_, w| w.tx_lsb_first().clear_bit().rx_lsb_first().clear_bit());

            me.reset_fifo();

            // Set LP I2C source clock
            unsafe { &*LP_CLKRST::PTR }
                .lpperi()
                .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

            // Configure LP I2C timing paramters. source_clk is ignored for LP_I2C in this
            // call

            let source_clk = 16_000_000;
            let bus_freq = frequency.raw();

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
                me.i2c.clk_conf().modify(|_, w| {
                    w.sclk_sel()
                        .clear_bit()
                        .sclk_div_num()
                        .bits((clkm_div - 1) as u8)
                });

                // scl period
                me.i2c
                    .scl_low_period()
                    .write(|w| w.scl_low_period().bits(scl_low_period as u16));

                me.i2c.scl_high_period().write(|w| {
                    w.scl_high_period()
                        .bits(scl_high_period as u16)
                        .scl_wait_high_period()
                        .bits(scl_wait_high_period as u8)
                });
                // sda sample
                me.i2c
                    .sda_hold()
                    .write(|w| w.time().bits(sda_hold_time as u16));
                me.i2c
                    .sda_sample()
                    .write(|w| w.time().bits(sda_sample_time as u16));

                // setup
                me.i2c
                    .scl_rstart_setup()
                    .write(|w| w.time().bits(scl_rstart_setup_time as u16));
                me.i2c
                    .scl_stop_setup()
                    .write(|w| w.time().bits(scl_stop_setup_time as u16));

                // hold
                me.i2c
                    .scl_start_hold()
                    .write(|w| w.time().bits(scl_start_hold_time as u16));
                me.i2c
                    .scl_stop_hold()
                    .write(|w| w.time().bits(scl_stop_hold_time as u16));

                me.i2c.to().write(|w| {
                    w.time_out_en()
                        .bit(time_out_en)
                        .time_out_value()
                        .variant(time_out_value.try_into().unwrap())
                });
            }

            // Enable SDA and SCL filtering. This configuration matches the HP I2C filter
            // config

            me.i2c
                .filter_cfg()
                .modify(|_, w| unsafe { w.sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });
            me.i2c
                .filter_cfg()
                .modify(|_, w| unsafe { w.scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });

            me.i2c
                .filter_cfg()
                .modify(|_, w| w.sda_filter_en().set_bit());
            me.i2c
                .filter_cfg()
                .modify(|_, w| w.scl_filter_en().set_bit());

            // Configure the I2C master to send a NACK when the Rx FIFO count is full
            me.i2c.ctr().modify(|_, w| w.rx_full_ack_level().set_bit());

            // Synchronize the config register values to the LP I2C peripheral clock
            me.lp_i2c_update();

            me
        }

        /// Update I2C configuration
        fn lp_i2c_update(&self) {
            self.i2c.ctr().modify(|_, w| w.conf_upgate().set_bit());
        }

        fn reset_fifo(&self) {
            self.i2c
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().set_bit());

            self.i2c
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().clear_bit());

            self.i2c
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().set_bit());

            self.i2c
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().clear_bit());
        }
    }
}
