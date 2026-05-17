//! D+ / D- pin configuration for full-speed USB OTG (analog pad drive strength).

use crate::{gpio::Pin, peripherals::IO_MUX};

/// USB D+ (data plus) pin for full-speed OTG.
pub trait UsbFsDp: crate::private::Sealed {
    #[doc(hidden)]
    fn configure(&self);
}

/// USB D- (data minus) pin for full-speed OTG.
pub trait UsbFsDm: crate::private::Sealed {
    #[doc(hidden)]
    fn configure(&self);
}

fn set_high_drive(pin: &impl Pin) {
    IO_MUX::regs()
        .gpio(pin.number() as usize)
        .modify(|_, w| unsafe { w.fun_drv().bits(3) });
}

for_each_analog_function! {
    (USB_FS_DM, $gpio:ident) => {
        impl UsbFsDm for crate::peripherals::$gpio<'_> {
            fn configure(&self) {
                set_high_drive(self);
            }
        }
    };
    (USB_FS_DP, $gpio:ident) => {
        impl UsbFsDp for crate::peripherals::$gpio<'_> {
            fn configure(&self) {
                set_high_drive(self);
            }
        }
    };
}
