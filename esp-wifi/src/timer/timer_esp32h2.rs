#[cfg(feature = "ble")]
use crate::{
    binary,
    hal::{interrupt, peripherals::Interrupt},
};

pub(crate) fn setup_radio_isr() {
    #[cfg(feature = "ble")]
    {
        unwrap!(interrupt::enable(
            Interrupt::LP_BLE_TIMER,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::BT_MAC,
            interrupt::Priority::Priority1
        ));
    }
}

pub(crate) fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    {
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::LP_BLE_TIMER);
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::BT_MAC);
    }
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn LP_BLE_TIMER() {
    unsafe {
        trace!("LP_TIMER interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_3;

        trace!("interrupt LP_TIMER {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt LP_TIMER call");

            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("LP_TIMER done");
        }

        trace!("interrupt LP_TIMER done");
    };
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn BT_MAC() {
    unsafe {
        trace!("BT_MAC interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_15;

        trace!("interrupt BT_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt BT_MAC call");

            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("BT_MAC done");
        }

        trace!("interrupt BT_MAC done");
    };
}
