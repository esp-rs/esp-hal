pub(crate) fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    unsafe {
        use crate::hal::peripherals::BT;
        BT::steal().disable_rwbt_interrupt();
        BT::steal().disable_bb_interrupt();
    }
}

#[cfg(feature = "ble")]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
fn Software0() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_7;
        trace!("interrupt Software0 {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "wifi")]
#[unsafe(no_mangle)]
extern "C" fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[unsafe(no_mangle)]
extern "C" fn RWBT() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBT {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[unsafe(no_mangle)]
extern "C" fn RWBLE() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBLE {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[unsafe(no_mangle)]
extern "C" fn BT_BB() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_8;
        trace!("interrupt BT_BB {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}
