use crate::hal::{interrupt, macros::interrupt, peripherals};

pub fn setup_radio_isr() {
    #[cfg(feature = "wifi")]
    {
        unwrap!(interrupt::enable(
            peripherals::Interrupt::WIFI_MAC,
            interrupt::Priority::Priority1,
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::WIFI_PWR,
            interrupt::Priority::Priority1,
        ));
    }

    // ble not supported
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}
