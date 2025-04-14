//! # Universal Asynchronous Receiver/Transmitter (UART)
//!
//! ## Overview
//!
//! The UART is a hardware peripheral which handles communication using serial
//! interfaces. This peripheral provides a cheap and ubiquitous method for full-
//! and half-duplex communication between devices.
//!
//! ## Configuration
//!
//! The usual setting such as baud rate, data bits, parity, and stop bits can
//! easily be configured. See the [config] module documentation for more
//! information.
//!
//! ## Usage
//!
//! The UART driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes the [embedded-hal], [embedded-hal-nb], and
//! [embedded-io] traits.
//!
//! ## Examples
//!
//! ```rust
//! fn main(mut uart: LpUart) -> ! {
//!     loop {
//!         writeln!(uart, "Hello, world!").ok();
//!         esp_lp_hal::delay::Delay.delay_ms(1000);
//!     }
//! }
//! ```
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-hal-nb]: https://docs.rs/embedded-hal-nb/latest/embedded_hal_nb/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/

use crate::pac::LP_UART;

const UART_FIFO_SIZE: u16 = 128;

#[doc(hidden)]
pub unsafe fn conjure() -> LpUart { unsafe {
    LpUart {
        uart: LP_UART::steal(),
    }
}}

/// UART Error
#[derive(Debug)]
pub enum Error {}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        embedded_hal_nb::serial::ErrorKind::Other
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// UART configuration
pub mod config {
    /// Number of data bits
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub enum DataBits {
        /// 5 data bits
        DataBits5 = 0,
        /// 6 data bits
        DataBits6 = 1,
        /// 7 data bits
        DataBits7 = 2,
        /// 8 data bits
        #[default]
        DataBits8 = 3,
    }

    /// Parity check
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub enum Parity {
        /// No parity
        #[default]
        ParityNone = 0,
        /// Even parity
        ParityEven = 1,
        /// Odd parity
        ParityOdd  = 2,
    }

    /// Number of stop bits
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub enum StopBits {
        /// 1 stop bit
        #[default]
        Stop1   = 1,
        /// 1.5 stop bits
        Stop1p5 = 2,
        /// 2 stop bits
        Stop2   = 3,
    }

    /// UART configuration
    #[derive(Debug, Clone, Copy)]
    pub struct Config {
        baudrate: u32,
        data_bits: DataBits,
        parity: Parity,
        stop_bits: StopBits,
    }

    impl Config {
        /// Configure the UART's baud rate
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        /// Configure the UART to use no parity
        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        /// Configure the UART to use even parity
        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        /// Configure the UART to use odd parity
        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        /// Configure the UART's data bits
        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        /// Configure the UART's stop bits
        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: Default::default(),
                parity: Default::default(),
                stop_bits: Default::default(),
            }
        }
    }
}

/// LP-UART driver
pub struct LpUart {
    uart: LP_UART,
}

impl LpUart {
    /// Read a single byte from the UART in a non-blocking manner.
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        if self.rx_fifo_count() > 0 {
            let byte = self.uart.fifo().read().rxfifo_rd_byte().bits();
            Ok(byte)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Write a single byte to the UART in a non-blocking manner.
    pub fn write_byte(&mut self, byte: u8) -> nb::Result<(), Error> {
        if self.tx_fifo_count() < UART_FIFO_SIZE {
            self.uart
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Write one or more byte to the UART, blocking until the write has
    /// completed.
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        let count = data.len();

        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))?;

        Ok(count)
    }

    /// Flush the UART's transmit buffer in a non-blocking manner.
    pub fn flush_tx(&mut self) -> nb::Result<(), Error> {
        if self.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn rx_fifo_count(&mut self) -> u16 {
        self.uart.status().read().rxfifo_cnt().bits().into()
    }

    fn tx_fifo_count(&mut self) -> u16 {
        self.uart.status().read().txfifo_cnt().bits().into()
    }

    fn is_tx_idle(&self) -> bool {
        self.uart.fsm_status().read().st_utx_out().bits() == 0
    }
}

impl core::fmt::Write for LpUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::ErrorType for LpUart {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Read for LpUart {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Write for LpUart {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::ErrorType for LpUart {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Read for LpUart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        while self.rx_fifo_count() == 0 {
            // Block until we have received at least one byte
        }

        let mut count = 0;
        while self.rx_fifo_count() > 0 && count < buf.len() {
            buf[count] = self.uart.fifo().read().rxfifo_rd_byte().bits();
            count += 1;
        }

        Ok(count)
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::ReadReady for LpUart {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx_fifo_count() > 0)
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Write for LpUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            match self.flush_tx() {
                Ok(_) => break,
                Err(nb::Error::WouldBlock) => { /* Wait */ }
                #[allow(unreachable_patterns)]
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }

        Ok(())
    }
}
