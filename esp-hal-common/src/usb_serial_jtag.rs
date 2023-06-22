//! USB Serial JTAG peripheral driver

use core::convert::Infallible;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{usb_device::RegisterBlock, USB_DEVICE},
    system::PeripheralClockControl,
};

/// USB Serial JTAG driver
pub struct UsbSerialJtag<'d> {
    usb_serial: PeripheralRef<'d, USB_DEVICE>,
}

/// Custom USB serial error type
type Error = Infallible;

impl<'d> UsbSerialJtag<'d> {
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(
        usb_serial: impl Peripheral<P = USB_DEVICE> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(usb_serial);

        peripheral_clock_control.enable(crate::system::Peripheral::Sha);

        let mut dev = Self { usb_serial };
        dev.usb_serial.disable_rx_interrupts();
        dev.usb_serial.disable_tx_interrupts();

        dev
    }

    /// Write data to the serial output in chunks of up to 64 bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let reg_block = self.usb_serial.register_block();

        for chunk in data.chunks(64) {
            unsafe {
                for &b in chunk {
                    reg_block.ep1.write(|w| w.rdwr_byte().bits(b.into()))
                }
                reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

                while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
                    // wait
                }
            }
        }

        Ok(())
    }

    /// Write data to the serial output in a non-blocking manner
    /// Requires manual flushing (automatically flushed every 64 bytes)
    pub fn write_byte_nb(&mut self, word: u8) -> nb::Result<(), Error> {
        let reg_block = self.usb_serial.register_block();

        if reg_block
            .ep1_conf
            .read()
            .serial_in_ep_data_free()
            .bit_is_set()
        {
            // the FIFO is not full
            unsafe {
                reg_block.ep1.write(|w| w.rdwr_byte().bits(word.into()));
            }

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Flush the output FIFO and block until it has been sent
    pub fn flush_tx(&mut self) -> Result<(), Error> {
        let reg_block = self.usb_serial.register_block();
        reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

        while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
            // wait
        }

        Ok(())
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> nb::Result<(), Error> {
        let reg_block = self.usb_serial.register_block();
        reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

        if reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let reg_block = self.usb_serial.register_block();

        // Check if there are any bytes to read
        if reg_block
            .ep1_conf
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
        {
            let value = reg_block.ep1.read().rdwr_byte().bits();

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Listen for RX-PACKET-RECV interrupts
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().clear_bit());
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        let reg_block = unsafe { &*USB_DEVICE::PTR };
        reg_block
            .int_st
            .read()
            .serial_out_recv_pkt_int_st()
            .bit_is_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        let reg_block = unsafe { &*USB_DEVICE::PTR };

        reg_block
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }
}

/// USB Serial JTAG peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn disable_tx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_in_empty_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_in_empty_int_clr().set_bit())
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }

    fn get_rx_fifo_count(&self) -> u16 {
        let ep0_state = self.register_block().in_ep0_st.read();
        let wr_addr = ep0_state.in_ep0_wr_addr().bits();
        let rd_addr = ep0_state.in_ep0_rd_addr().bits();

        (wr_addr - rd_addr).into()
    }

    fn get_tx_fifo_count(&self) -> u16 {
        let ep1_state = self.register_block().in_ep1_st.read();
        let wr_addr = ep1_state.in_ep1_wr_addr().bits();
        let rd_addr = ep1_state.in_ep1_rd_addr().bits();

        (wr_addr - rd_addr).into()
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl core::fmt::Write for UsbSerialJtag<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl embedded_hal::serial::Read<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl embedded_hal::serial::Write<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::serial::ErrorType for UsbSerialJtag<'_> {
    type Error = Error;
}

#[cfg(feature = "eh1")]
impl embedded_hal_nb::serial::Read for UsbSerialJtag<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_nb::serial::Write for UsbSerialJtag<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}
