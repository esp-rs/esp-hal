#[cfg(any(feature = "wifi", feature = "ble"))]
#[allow(unused_imports)]
use crate::hal::{interrupt, peripherals};

pub fn setup_radio_isr() {
    // wifi enabled in set_isr
    #[cfg(feature = "ble")]
    {
        unwrap!(interrupt::enable(
            peripherals::Interrupt::RWBT,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::BT_BB,
            interrupt::Priority::Priority1,
        ));

        // It's a mystery why these interrupts are enabled now since it worked without
        // this before Now at least without disabling these nothing will work
        interrupt::disable(crate::hal::Cpu::ProCpu, peripherals::Interrupt::ETH_MAC);
        interrupt::disable(crate::hal::Cpu::ProCpu, peripherals::Interrupt::UART0);
    }
}

pub fn shutdown_radio_isr() {
    #[cfg(feature = "ble")]
    {
        interrupt::disable(crate::hal::Cpu::ProCpu, peripherals::Interrupt::RWBT);
        interrupt::disable(crate::hal::Cpu::ProCpu, peripherals::Interrupt::BT_BB);

        unwrap!(interrupt::enable(
            peripherals::Interrupt::ETH_MAC,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::UART0,
            interrupt::Priority::Priority1
        ));
    }
}

#[cfg(feature = "ble")]
#[allow(non_snake_case)]
#[no_mangle]
fn Software0(_level: u32) {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_7;
        trace!("interrupt Software0 {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "wifi")]
#[no_mangle]
extern "C" fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn RWBT() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBT {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn RWBLE() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBLE {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[no_mangle]
extern "C" fn BT_BB() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_8;
        trace!("interrupt BT_BB {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}
