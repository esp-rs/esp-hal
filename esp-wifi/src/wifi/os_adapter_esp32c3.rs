use crate::hal::{
    interrupt,
    peripherals::{self, Interrupt},
    riscv,
};

pub(crate) fn chip_ints_on(mask: u32) {
    unsafe {
        (*peripherals::INTERRUPT_CORE0::PTR)
            .cpu_int_enable()
            .modify(|r, w| w.bits(r.bits() | mask));
    }
}

pub(crate) fn chip_ints_off(mask: u32) {
    unsafe {
        (*peripherals::INTERRUPT_CORE0::PTR)
            .cpu_int_enable()
            .modify(|r, w| w.bits(r.bits() & !mask));
    }
}

pub(crate) unsafe extern "C" fn wifi_int_disable(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
) -> u32 {
    let res = if riscv::register::mstatus::read().mie() {
        1
    } else {
        0
    };
    riscv::interrupt::disable();

    trace!(
        "wifi_int_disable wifi_int_mux {:?} - return {}",
        wifi_int_mux,
        res,
    );

    res
}

pub(crate) unsafe extern "C" fn wifi_int_restore(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
    tmp: u32,
) {
    trace!(
        "wifi_int_restore wifi_int_mux {:?} tmp {}",
        wifi_int_mux,
        tmp
    );

    if tmp == 1 {
        riscv::interrupt::enable();
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
pub unsafe extern "C" fn set_isr(
    n: i32,
    f: *mut crate::binary::c_types::c_void,
    arg: *mut crate::binary::c_types::c_void,
) {
    trace!("set_isr - interrupt {} function {:?} arg {:?}", n, f, arg);

    match n {
        0 => {
            crate::wifi::ISR_INTERRUPT_1 = (f, arg);
        }
        1 => {
            crate::wifi::ISR_INTERRUPT_1 = (f, arg);
        }
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }

    #[cfg(feature = "wifi")]
    {
        unwrap!(interrupt::enable(
            Interrupt::WIFI_MAC,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::WIFI_PWR,
            interrupt::Priority::Priority1
        ));
    }
}
