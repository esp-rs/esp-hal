#[unsafe(no_mangle)]
extern "C" fn LP_TIMER() {
    unsafe {
        trace!("LP_TIMER interrupt");

        let (fnc, arg) = ISR_INTERRUPT_7;

        trace!("interrupt LP_TIMER {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt LP_TIMER call");

            let fnc: fn(*mut c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("LP_TIMER done");
        }

        trace!("interrupt LP_TIMER done");
    };
}

#[unsafe(no_mangle)]
extern "C" fn BT_MAC() {
    unsafe {
        trace!("BT_MAC interrupt");

        let (fnc, arg) = ISR_INTERRUPT_4;

        trace!("interrupt BT_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt BT_MAC call");

            let fnc: fn(*mut c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("BT_MAC done");
        }

        trace!("interrupt BT_MAC done");
    };
}

pub(crate) fn shutdown_ble_isr() {
    unsafe {
        use crate::hal::peripherals::BT;
        BT::steal().disable_lp_timer_interrupt_on_all_cores();
        BT::steal().disable_mac_interrupt_on_all_cores();
    }
}
