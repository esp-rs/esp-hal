use crate::hal::{interrupt, peripherals};

pub(crate) fn chip_ints_on(mask: u32) {
    unsafe {
        (*peripherals::INTPRI::PTR)
            .cpu_int_enable
            .modify(|r, w| w.bits(r.bits() | mask));
    }
}

pub(crate) fn chip_ints_off(mask: u32) {
    unsafe {
        (*peripherals::INTPRI::PTR)
            .cpu_int_enable
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

pub(crate) unsafe extern "C" fn regdma_link_set_write_wait_content_dummy(
    _arg1: *mut esp_wifi_sys::c_types::c_void,
    _arg2: u32,
    _arg3: u32,
) {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_find_link_by_id_dummy(
    _arg1: esp_wifi_sys::c_types::c_int,
) -> *mut esp_wifi_sys::c_types::c_void {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_entries_create_dummy(
    _arg1: *const esp_wifi_sys::c_types::c_void,
    _arg2: esp_wifi_sys::c_types::c_int,
    _arg3: esp_wifi_sys::c_types::c_int,
    _arg4: esp_wifi_sys::c_types::c_int,
) -> esp_wifi_sys::c_types::c_int {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_entries_destroy_dummy(
    _arg1: esp_wifi_sys::c_types::c_int,
) {
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
}
