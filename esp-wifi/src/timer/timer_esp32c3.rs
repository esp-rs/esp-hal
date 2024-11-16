#[cfg(any(feature = "wifi", feature = "ble"))]
#[allow(unused_imports)]
use crate::{
    binary,
    hal::{interrupt, peripherals::Interrupt},
};

pub(crate) fn setup_radio_isr() {
    // wifi enabled in set_isr
    #[cfg(feature = "ble")]
    {
        unwrap!(interrupt::enable(
            Interrupt::RWBT,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::RWBLE,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::BT_BB,
            interrupt::Priority::Priority1
        ));
    }
}

pub(crate) fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    {
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::RWBT);
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::RWBLE);
        interrupt::disable(crate::hal::Cpu::ProCpu, Interrupt::BT_BB);
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
extern "C" fn RWBT() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt RWBT {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 5 done");
    };
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn RWBLE() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION8;

        trace!("interrupt RWBLE {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 5 done");
    };
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn BT_BB(_trap_frame: &mut crate::hal::interrupt::TrapFrame) {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt BT_BB {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 8 done");
    };
}
