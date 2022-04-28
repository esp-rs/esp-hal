#[cfg(feature = "esp32c3")]
const USB_SERIAL_JTAG_FIFO_REG: usize = 0x6004_3000;
#[cfg(feature = "esp32c3")]
const USB_SERIAL_JTAG_CONF_REG: usize = 0x6004_3004;

#[cfg(feature = "esp32s3")]
const USB_SERIAL_JTAG_FIFO_REG: usize = 0x6003_8000;
#[cfg(feature = "esp32s3")]
const USB_SERIAL_JTAG_CONF_REG: usize = 0x6003_8004;

pub struct UsbSerialJtag;

impl core::fmt::Write for UsbSerialJtag {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        unsafe {
            let fifo = USB_SERIAL_JTAG_FIFO_REG as *mut u32;
            let conf = USB_SERIAL_JTAG_CONF_REG as *mut u32;

            // TODO: 64 byte chunks max
            for chunk in s.as_bytes().chunks(32) {
                for &b in chunk {
                    fifo.write_volatile(b as u32);
                }
                conf.write_volatile(0b001);

                while conf.read_volatile() & 0b011 == 0b000 {
                    // wait
                }
            }

            core::fmt::Result::Ok(())
        }
    }
}
