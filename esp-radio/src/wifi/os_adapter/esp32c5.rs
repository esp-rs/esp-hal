use crate::{
    hal::{interrupt::Priority, peripherals::WIFI},
    interrupt_dispatch::Handler,
    sys::c_types::{c_int, c_void},
};

static ISR_INTERRUPT_1: Handler = Handler::new();

pub(crate) fn chip_ints_on(_mask: u32) {
    // from docs: Enable Wi-Fi interrupt / mask - No mean
    // i.e. nothing to do - we already enabled the WIFI interrupt
}

pub(crate) fn chip_ints_off(_mask: u32) {
    // from docs: Disable Wi-Fi interrupt / mask - No mean
    // leaving the interrupt enabled seems to not cause issues
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

pub(crate) unsafe extern "C" fn regdma_link_set_write_wait_content_dummy(
    _arg1: *mut c_void,
    _arg2: u32,
    _arg3: u32,
) {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_find_link_by_id_dummy(_arg1: c_int) -> *mut c_void {
    todo!()
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
