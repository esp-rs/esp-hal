#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(non_snake_case)]
use log::trace;

const DR_REG_DPORT_BASE: u32 = 0x3ff00000;
const DPORT_WIFI_CLK_EN_REG: u32 = DR_REG_DPORT_BASE + 0x0CC;
const DPORT_WIFI_CLK_WIFI_EN: u32 = 0x00000406;
const DPORT_WIFI_CLK_WIFI_EN_V: u32 = 0x406;
const DPORT_WIFI_CLK_WIFI_EN_S: u32 = 0;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = (DPORT_WIFI_CLK_WIFI_EN_V) << (DPORT_WIFI_CLK_WIFI_EN_S);

pub(crate) fn chip_ints_on(mask: u32) {
    trace!("chip_ints_on esp32");
    unsafe {
        xtensa_lx::interrupt::enable_mask(1 << 0);
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

pub(crate) unsafe extern "C" fn phy_common_clock_disable() {
    crate::common_adapter::chip_specific::phy_disable_clock();
}

pub(crate) unsafe extern "C" fn phy_common_clock_enable() {
    crate::common_adapter::chip_specific::phy_enable_clock();
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

pub(crate) unsafe extern "C" fn wifi_clock_enable() {
    trace!("wifi_clock_enable");

    let ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
    let old = ptr.read_volatile();
    ptr.write_volatile(old | DPORT_WIFI_CLK_WIFI_EN_M);
}

pub(crate) unsafe extern "C" fn wifi_clock_disable() {
    trace!("wifi_clock_disable");

    let ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
    let old = ptr.read_volatile();
    ptr.write_volatile(old & !DPORT_WIFI_CLK_WIFI_EN_M);
}
