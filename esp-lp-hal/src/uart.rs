//! Low-power UART driver

use crate::pac::LP_UART;

const UART_FIFO_SIZE: u16 = 128;

#[doc(hidden)]
pub unsafe fn conjure() -> LpUart {
    LpUart {
        uart: LP_UART::steal(),
    }
}

#[derive(Debug)]
pub enum Error {}

/// UART configuration
pub mod config {
    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum DataBits {
        DataBits5 = 0,
        DataBits6 = 1,
        DataBits7 = 2,
        DataBits8 = 3,
    }

    /// Parity check
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Parity {
        ParityNone = 0,
        ParityEven = 1,
        ParityOdd  = 2,
    }

    /// Number of stop bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum StopBits {
        /// 1 stop bit
        STOP1   = 1,
        /// 1.5 stop bits
        STOP1P5 = 2,
        /// 2 stop bits
        STOP2   = 3,
    }

    /// UART configuration
    #[derive(Debug, Copy, Clone)]
    pub struct Config {
        pub baudrate: u32,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
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
