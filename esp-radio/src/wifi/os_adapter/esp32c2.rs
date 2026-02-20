use crate::{
    hal::{
        interrupt::Priority,
        peripherals::{INTERRUPT_CORE0, WIFI},
    },
    interrupt_dispatch::Handler,
    sys::c_types::c_void,
};

static ISR_INTERRUPT_1: Handler = Handler::new();

pub(crate) fn chip_ints_on(mask: u32) {
    unsafe {
        INTERRUPT_CORE0::regs()
            .cpu_int_enable()
            .modify(|r, w| w.bits(r.bits() | mask));
    }
}

pub(crate) fn chip_ints_off(mask: u32) {
    unsafe {
        INTERRUPT_CORE0::regs()
            .cpu_int_enable()
            .modify(|r, w| w.bits(r.bits() & !mask));
    }
}

pub(crate) unsafe extern "C" fn set_intr(
    _cpu_no: i32,
    _intr_source: u32,
    _intr_num: u32,
    _intr_prio: i32,
) {
    // this gets called with
    // INFO - set_intr 0 2 1 1 (WIFI_PWR)
    // INFO - set_intr 0 0 1 1 (WIFI_MAC)

    // we do nothing here since all the interrupts are already
    // configured in `setup_timer_isr` and messing with the interrupts will
    // get us into trouble
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
        0 | 1 => ISR_INTERRUPT_1.set(f, arg),
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }

    unsafe {
        WIFI::steal().enable_mac_interrupt(Priority::Priority1);
        WIFI::steal().enable_pwr_interrupt(Priority::Priority1);
    }
}

// For ESP32-C2 < ECO4
#[unsafe(no_mangle)]
unsafe extern "C" fn esp32c2_eco4_rom_ptr_init() {
    // Do not remove, stub to overwrite weak link in Wi-Fi Lib
    //
    // Otherwise you will see:
    //
    // undefined symbol: s_pm_beacon_offset_ptr
    // undefined symbol: g_authmode_threshold_failure_ptr
    // undefined symbol: len_dh_ie_ptr
    // undefined symbol: s_tbttstart_ptr
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn WIFI_MAC() {
    ISR_INTERRUPT_1.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn WIFI_PWR() {
    ISR_INTERRUPT_1.dispatch();
}

pub(crate) fn shutdown_wifi_isr() {
    unsafe {
        WIFI::steal().disable_mac_interrupt();
        WIFI::steal().disable_pwr_interrupt();
    }
}
