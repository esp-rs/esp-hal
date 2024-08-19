#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(non_snake_case)]

use crate::hal::{interrupt, peripherals};

const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x406;

pub(crate) fn chip_ints_on(mask: u32) {
    unsafe { crate::hal::xtensa_lx::interrupt::enable_mask(mask) };
}

pub(crate) fn chip_ints_off(mask: u32) {
    crate::hal::xtensa_lx::interrupt::disable_mask(mask);
}

pub(crate) unsafe extern "C" fn wifi_int_disable(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
) -> u32 {
    core::mem::transmute(critical_section::acquire())
}

pub(crate) unsafe extern "C" fn wifi_int_restore(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
    tmp: u32,
) {
    critical_section::release(core::mem::transmute::<u32, critical_section::RestoreState>(
        tmp,
    ))
}

pub(crate) unsafe extern "C" fn phy_common_clock_disable() {
    crate::common_adapter::chip_specific::phy_disable_clock();
}

pub(crate) unsafe extern "C" fn phy_common_clock_enable() {
    crate::common_adapter::chip_specific::phy_enable_clock();
}

pub(crate) unsafe extern "C" fn set_intr(
    _cpu_no: i32,
    intr_source: u32,
    intr_num: u32,
    _intr_prio: i32,
) {
    extern "C" {
        fn intr_matrix_set(cpu_no: u32, model_num: u32, intr_num: u32);
    }
    // Force to bind WiFi interrupt to CPU0
    intr_matrix_set(0, intr_source, intr_num);
}

pub(crate) unsafe extern "C" fn wifi_clock_enable() {
    let dport = &*crate::hal::peripherals::SYSTEM::ptr();
    dport.wifi_clk_en().modify(|r, w| {
        let old = r.bits();
        let new_bits = old | DPORT_WIFI_CLK_WIFI_EN_M;
        w.bits(new_bits)
    });
}

pub(crate) unsafe extern "C" fn wifi_clock_disable() {
    let dport = &*crate::hal::peripherals::SYSTEM::ptr();
    dport.wifi_clk_en().modify(|r, w| {
        let old = r.bits();
        let new_bits = old & !DPORT_WIFI_CLK_WIFI_EN_M;
        w.bits(new_bits)
    });
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
            peripherals::Interrupt::WIFI_MAC,
            interrupt::Priority::Priority1,
        ));
    }
}
