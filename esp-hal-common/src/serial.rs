//! UART driver

#[cfg(any(feature = "esp32", feature = "esp32s3"))]
use crate::pac::UART2;
use crate::pac::{uart0::RegisterBlock, UART0, UART1};

const UART_FIFO_SIZE: u16 = 128;

/// Custom serial error type
#[derive(Debug)]
pub enum Error {}

impl embedded_hal_1::serial::Error for Error {
    fn kind(&self) -> embedded_hal_1::serial::ErrorKind {
        embedded_hal_1::serial::ErrorKind::Other
    }
}

/// UART driver
pub struct Serial<T> {
    uart: T,
}

impl<T> Serial<T>
where
    T: Instance,
{
    /// Create a new UART instance
    pub fn new(uart: T) -> Result<Self, Error> {
        let mut serial = Serial { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        Ok(serial)
    }

    /// Return the raw interface to the underlying UART instance
    pub fn free(self) -> T {
        self.uart
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        if self.uart.get_tx_fifo_count() < UART_FIFO_SIZE {
            self.uart
                .register_block()
                .fifo
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush_tx(&self) -> nb::Result<(), Error> {
        if self.uart.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        if self.uart.get_rx_fifo_count() > 0 {
            let value = self
                .uart
                .register_block()
                .fifo
                .read()
                .rxfifo_rd_byte()
                .bits();

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// UART peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn disable_tx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.txfifo_empty_int_clr()
                .set_bit()
                .tx_brk_done_int_clr()
                .set_bit()
                .tx_brk_idle_done_int_clr()
                .set_bit()
                .tx_done_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.txfifo_empty_int_ena()
                .clear_bit()
                .tx_brk_done_int_ena()
                .clear_bit()
                .tx_brk_idle_done_int_ena()
                .clear_bit()
                .tx_done_int_ena()
                .clear_bit()
        });
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.rxfifo_full_int_clr()
                .set_bit()
                .rxfifo_ovf_int_clr()
                .set_bit()
                .rxfifo_tout_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.rxfifo_full_int_ena()
                .clear_bit()
                .rxfifo_ovf_int_ena()
                .clear_bit()
                .rxfifo_tout_int_ena()
                .clear_bit()
        });
    }

    fn get_tx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .txfifo_cnt()
            .bits()
            .into()
    }

    fn get_rx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .rxfifo_cnt()
            .bits()
            .into()
    }

    fn is_tx_idle(&self) -> bool {
        #[cfg(feature = "esp32")]
        let idle = self.register_block().status.read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(feature = "esp32"))]
        let idle = self.register_block().fsm_status.read().st_utx_out().bits() == 0x0u8;

        idle
    }

    fn is_rx_idle(&self) -> bool {
        #[cfg(feature = "esp32")]
        let idle = self.register_block().status.read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(feature = "esp32"))]
        let idle = self.register_block().fsm_status.read().st_urx_out().bits() == 0x0u8;

        idle
    }
}

impl Instance for UART0 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl Instance for UART1 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

#[cfg(any(feature = "esp32", feature = "esp32s3"))]
impl Instance for UART2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

#[cfg(feature = "ufmt")]
impl<T> ufmt_write::uWrite for Serial<T>
where
    T: Instance,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 4];
        self.write_bytes(ch.encode_utf8(&mut buffer).as_bytes())
    }
}

impl<T> core::fmt::Write for Serial<T>
where
    T: Instance,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<T> embedded_hal::serial::Write<u8> for Serial<T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T> embedded_hal::serial::Read<u8> for Serial<T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T> embedded_hal_1::serial::ErrorType for Serial<T> {
    type Error = Error;
}

impl<T> embedded_hal_1::serial::nb::Read for Serial<T>
where
    T: Instance,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T> embedded_hal_1::serial::nb::Write for Serial<T>
where
    T: Instance,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}
