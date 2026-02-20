pub(crate) fn shutdown_radio_isr() {
    // ble not supported
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

#[cfg(feature = "wifi")]
#[unsafe(no_mangle)]
extern "C" fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}
