//! # Universal Asynchronous Receiver/Transmitter (UART)

use crate::pac::LP_UART;

const UART_FIFO_SIZE: u16 = 128;

#[doc(hidden)]
pub unsafe fn conjure() -> LpUart {
    LpUart {
        uart: LP_UART::steal(),
    }
}

/// UART Error
#[derive(Debug)]
pub enum Error {}

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
    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        if self.get_rx_fifo_count() > 0 {
            let byte = self.uart.fifo().read().rxfifo_rd_byte().bits();
            Ok(byte)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_byte(&mut self, byte: u8) -> nb::Result<(), Error> {
        if self.get_tx_fifo_count() < UART_FIFO_SIZE {
            self.uart
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_bytes(&mut self, data: &[u8]) -> nb::Result<(), Error> {
        data.iter().try_for_each(|c| self.write_byte(*c))
    }

    fn flush_tx(&mut self) -> nb::Result<(), Error> {
        if self.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn get_rx_fifo_count(&mut self) -> u16 {
        self.uart.status().read().rxfifo_cnt().bits().into()
    }

    fn get_tx_fifo_count(&mut self) -> u16 {
        self.uart.status().read().txfifo_cnt().bits().into()
    }

    fn is_tx_idle(&self) -> bool {
        self.uart.fsm_status().read().st_utx_out().bits() == 0
    }
}

impl core::fmt::Write for LpUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::serial::Read<u8> for LpUart {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::serial::Write<u8> for LpUart {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}
