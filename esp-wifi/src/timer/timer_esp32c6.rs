use crate::hal::peripherals;
#[cfg(any(feature = "wifi", feature = "ble"))]
#[allow(unused_imports)]
use crate::{
    binary,
    hal::{interrupt, peripherals::Interrupt},
};

pub(crate) fn setup_radio_isr() {
    // make sure to disable WIFI_BB/MODEM_PERI_TIMEOUT by mapping it to CPU
    // interrupt 31 which is masked by default for some reason for this
    // interrupt, mapping it to 0 doesn't deactivate it
    let interrupt_core0 = unsafe { &*peripherals::INTERRUPT_CORE0::PTR };
    interrupt_core0
        .wifi_bb_intr_map()
        .write(|w| unsafe { w.wifi_bb_intr_map().bits(31) });
    interrupt_core0
        .modem_peri_timeout_intr_map()
        .write(|w| unsafe { w.modem_peri_timeout_intr_map().bits(31) });
}

pub(crate) fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    {
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::LP_TIMER);
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::BT_MAC);
    }
}

#[cfg(feature = "wifi")]
#[no_mangle]
extern "C" fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[cfg(feature = "wifi")]
#[no_mangle]
extern "C" fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn LP_TIMER() {
    unsafe {
        trace!("LP_TIMER interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_7;

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

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_4;

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
