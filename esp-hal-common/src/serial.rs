use embedded_hal::serial::{Read, Write};

#[cfg(any(feature = "32", feature = "32s3"))]
use crate::pac::UART2;
use crate::pac::{uart0::RegisterBlock, UART0, UART1};

const UART_FIFO_SIZE: u16 = 128;

#[derive(Debug)]
pub enum Error {}

pub struct Serial<T> {
    uart: T,
}

impl<T: Instance> Serial<T> {
    pub fn new(uart: T) -> Result<Self, Error> {
        let mut serial = Serial { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        Ok(serial)
    }
}

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

    fn is_tx_idle(&mut self) -> bool {
        #[cfg(feature = "32")]
        let idle = self.register_block().status.read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(feature = "32"))]
        let idle = self.register_block().fsm_status.read().st_utx_out().bits() == 0x0u8;

        idle
    }

    fn is_rx_idle(&mut self) -> bool {
        #[cfg(feature = "32")]
        let idle = self.register_block().status.read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(feature = "32"))]
        let idle = self.register_block().fsm_status.read().st_urx_out().bits() == 0x0u8;

        idle
    }
}

impl<T: Instance> Write<u8> for Serial<T> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.uart.get_tx_fifo_count() < UART_FIFO_SIZE {
            #[cfg(feature = "32")]
            self.uart
                .register_block()
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            #[cfg(not(feature = "32"))]
            self.uart
                .register_block()
                .fifo
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.uart.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T: Instance> Read<u8> for Serial<T> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.uart.get_rx_fifo_count() > 0 {
            #[cfg(feature = "32")]
            let value = self
                .uart
                .register_block()
                .fifo()
                .read()
                .rxfifo_rd_byte()
                .bits();

            #[cfg(not(feature = "32"))]
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

impl<T: Instance> core::fmt::Write for Serial<T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
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

#[cfg(any(feature = "32", feature = "32s3"))]
impl Instance for UART2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}
