use crate::{
    hal::{interrupt::Priority, peripherals::WIFI},
    sys::c_types::c_void,
};

pub(crate) fn chip_ints_on(mask: u32) {
    unsafe { crate::hal::xtensa_lx::interrupt::enable_mask(mask) };
}

pub(crate) fn chip_ints_off(mask: u32) {
    crate::hal::xtensa_lx::interrupt::disable_mask(mask);
}

pub(crate) unsafe extern "C" fn set_intr(
    _cpu_no: i32,
    intr_source: u32,
    intr_num: u32,
    _intr_prio: i32,
) {
    unsafe extern "C" {
        fn intr_matrix_set(cpu_no: u32, model_num: u32, intr_num: u32);
    }
    unsafe {
        // Force to bind WiFi interrupt to CPU0
        intr_matrix_set(0, intr_source, intr_num);
    }
}

pub(crate) unsafe extern "C" fn phy_common_clock_disable() {
    unsafe {
        crate::common_adapter::phy_disable_clock();
    }
}

pub(crate) unsafe extern "C" fn phy_common_clock_enable() {
    unsafe {
        crate::common_adapter::phy_enable_clock();
    }
}

/// **************************************************************************
/// Name: esp_set_isr
///
/// Description:
///   Register interrupt function
///
/// Input Parameters:
///   n   - Interrupt ID
///   f   - Interrupt function
///   arg - Function private data
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn set_isr(n: i32, f: *mut c_void, arg: *mut c_void) {
    trace!("set_isr - interrupt {} function {:?} arg {:?}", n, f, arg);

    match n {
        0 => unsafe {
            crate::wifi::ISR_INTERRUPT_1 = (f, arg);
        },
        1 => unsafe {
            crate::wifi::ISR_INTERRUPT_1 = (f, arg);
        },
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }

    unsafe {
        WIFI::steal().enable_mac_interrupt(Priority::Priority1);
        WIFI::steal().enable_pwr_interrupt(Priority::Priority1);
    }
}

#[unsafe(no_mangle)]
extern "C" fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[unsafe(no_mangle)]
extern "C" fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

pub(crate) fn shutdown_wifi_isr() {
    unsafe {
        WIFI::steal().disable_mac_interrupt();
        WIFI::steal().disable_pwr_interrupt();
    }
}
