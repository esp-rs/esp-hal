#[cfg(feature = "ble")]
use crate::{hal::peripherals, sys};

pub(crate) fn setup_radio_isr() {
    // no-op
}

pub(crate) fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    unsafe {
        peripherals::BT::steal().disable_lp_timer_interrupt();
        peripherals::BT::steal().disable_mac_interrupt();
    }
}

#[cfg(feature = "ble")]
#[unsafe(no_mangle)]
extern "C" fn LP_BLE_TIMER() {
    unsafe {
        trace!("LP_TIMER interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_3;

        trace!("interrupt LP_TIMER {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt LP_TIMER call");

            let fnc: fn(*mut sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("LP_TIMER done");
        }

        trace!("interrupt LP_TIMER done");
    };
}

#[cfg(feature = "ble")]
#[unsafe(no_mangle)]
extern "C" fn BT_MAC() {
    unsafe {
        trace!("BT_MAC interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_15;

        trace!("interrupt BT_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt BT_MAC call");

            let fnc: fn(*mut sys::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("BT_MAC done");
        }

        trace!("interrupt BT_MAC done");
    };
}
