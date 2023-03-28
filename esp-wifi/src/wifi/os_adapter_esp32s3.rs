#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(non_snake_case)]
use log::trace;

pub(crate) fn chip_ints_on(mask: u32) {
    trace!("chip_ints_on esp32");
    unsafe {
        esp32s3_hal::xtensa_lx::interrupt::enable_mask(1 << 0);
    }
}

pub(crate) unsafe extern "C" fn wifi_int_disable(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
) -> u32 {
    trace!("wifi_int_disable() esp32");
    core::mem::transmute(critical_section::acquire())
}

pub(crate) unsafe extern "C" fn wifi_int_restore(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
    tmp: u32,
) {
    trace!("wifi_int_restore() esp32");
    critical_section::release(core::mem::transmute(tmp))
}

pub(crate) unsafe extern "C" fn set_intr(
    cpu_no: i32,
    intr_source: u32,
    intr_num: u32,
    intr_prio: i32,
) {
    extern "C" {
        fn intr_matrix_set(cpu_no: u32, model_num: u32, intr_num: u32);
    }
    // Force to bind WiFi interrupt to CPU0
    intr_matrix_set(0, intr_source, intr_num);
}
