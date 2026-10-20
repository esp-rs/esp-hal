use crate::{
    hal::{interrupt::Priority, peripherals::WIFI, ram},
    interrupt_dispatch::Handler,
    sys::c_types::{c_int, c_void},
};

static ISR_INTERRUPT_1: Handler = Handler::new();

pub(crate) fn chip_ints_on(mask: u32) {
    // set_intr called with intr_num = 1 -> mask = 1 << 1 == 2
    trace!("chip_ints_on - mask {:#x}", mask);
    if mask & 2 == 2 {
        unsafe {
            WIFI::steal().enable_mac_interrupt(Priority::Priority1);
            WIFI::steal().enable_pwr_interrupt(Priority::Priority1);
        }
    }
}

pub(crate) fn chip_ints_off(mask: u32) {
    trace!("chip_ints_off - mask {:#x}", mask);
    if mask & 2 == 2 {
        unsafe {
            WIFI::steal().disable_mac_interrupt_on_all_cores();
            WIFI::steal().disable_pwr_interrupt_on_all_cores();
        }
    }
}

pub(crate) unsafe extern "C" fn set_intr(
    cpu_no: i32,
    intr_source: u32,
    intr_num: u32,
    intr_prio: i32,
) {
    trace!(
        "set_intr - core {} interrupt {} num {:?} prio {:?}",
        cpu_no, intr_source, intr_num, intr_prio
    );
    // These are expected to be direct-bound, but we don't do that for now.
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

pub unsafe extern "C" fn set_isr(n: i32, f: *mut c_void, arg: *mut c_void) {
    trace!("set_isr - interrupt {} function {:?} arg {:?}", n, f, arg);

    match n {
        1 => ISR_INTERRUPT_1.set(f, arg),
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn MODEM_WIFI_MAC() {
    ISR_INTERRUPT_1.dispatch();
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn MODEM_WIFI_PWR() {
    ISR_INTERRUPT_1.dispatch();
}

pub(crate) fn shutdown_wifi_isr() {
    unsafe {
        WIFI::steal().disable_mac_interrupt_on_all_cores();
        WIFI::steal().disable_pwr_interrupt_on_all_cores();
    }
}
