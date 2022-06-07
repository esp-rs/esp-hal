use crate::pac::USB_DEVICE;

pub struct UsbSerialJtag;

impl core::fmt::Write for UsbSerialJtag {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let reg_block = unsafe { &*USB_DEVICE::PTR };

        // TODO: 64 byte chunks max
        for chunk in s.as_bytes().chunks(32) {
            unsafe {
                for &b in chunk {
                    reg_block.ep1.write(|w| w.bits(b.into()))
                }
                reg_block.ep1_conf.write(|w| w.bits(0b001));

                while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
                    // wait
                }
            }
        }

        core::fmt::Result::Ok(())
    }
}
