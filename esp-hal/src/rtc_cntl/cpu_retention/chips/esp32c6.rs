//! ESP32-C6 retention buffer layout and CPU-domain device regions.

use crate::{
    peripherals::{CLINT, EXTMEM, INTPRI, PLIC_MX},
    rtc_cntl::cpu_retention::{
        device_regs::DeviceRegion,
        frames::c6_h2::{CRITICAL_FRAME_SIZE, NON_CRITICAL_FRAME_SIZE},
    },
};

/// `INTPRI_CORE0_CPU_INT_ENABLE_REG` through `INTPRI_RND_ECO_LOW_REG`, at offsets 0x0 to 0xb0.
const INTPRI_WORDS: usize = 45;
/// `PLIC_*XINT_ENABLE_REG` through `PLIC_*XINT_CLAIM_REG`, at offsets 0x0 to 0x94.
const PLIC_INT_WORDS: usize = 38;
/// `CLINT_*INT_SIP_REG` through `CLINT_*INT_*TIMECMP_H_REG`, at offsets 0x0 to 0x14.
const CLINT_WORDS: usize = 6;

// The PAC describes the machine-mode PLIC block only. These come from
// `components/soc/esp32c6/register/soc/plic_reg.h`.
const PLIC_MXINT_CONF: *const u32 = 0x2000_13fc as *const u32;
const PLIC_UXINT_ENABLE: *const u32 = 0x2000_1400 as *const u32;
const PLIC_UXINT_CONF: *const u32 = 0x2000_17fc as *const u32;

/// The regions that `esp32c6/sleep_cpu_dynamic.c:45-78` declares.
#[crate::ram]
pub(crate) fn regions() -> [DeviceRegion; 10] {
    let intpri = INTPRI::regs();
    let extmem = EXTMEM::regs();
    let plic_mx = PLIC_MX::regs();
    let clint = CLINT::regs();

    [
        DeviceRegion::new(intpri.cpu_int_enable().as_ptr(), INTPRI_WORDS),
        DeviceRegion::new(intpri.rnd_eco_high().as_ptr(), 1),
        DeviceRegion::new(extmem.l1_cache_ctrl().as_ptr(), 1),
        DeviceRegion::new(extmem.l1_cache_wrap_around_ctrl().as_ptr(), 1),
        DeviceRegion::new(plic_mx.mxint_enable().as_ptr(), PLIC_INT_WORDS),
        DeviceRegion::new(PLIC_MXINT_CONF, 1),
        DeviceRegion::new(PLIC_UXINT_ENABLE, PLIC_INT_WORDS),
        DeviceRegion::new(PLIC_UXINT_CONF, 1),
        DeviceRegion::new(clint.msip().as_ptr(), CLINT_WORDS),
        DeviceRegion::new(clint.usip().as_ptr(), CLINT_WORDS),
    ]
}

pub(crate) const DEVICE_REGION_WORDS: usize =
    INTPRI_WORDS + 1 + 2 + 2 * (PLIC_INT_WORDS + 1) + 2 * CLINT_WORDS;

const _: () = ::core::assert!(DEVICE_REGION_WORDS == 138);

pub(crate) const CRITICAL_FRAME_OFFSET: usize = 0;
pub(crate) const NON_CRITICAL_FRAME_OFFSET: usize = CRITICAL_FRAME_OFFSET + CRITICAL_FRAME_SIZE;
pub(crate) const DEVICE_REGIONS_OFFSET: usize = NON_CRITICAL_FRAME_OFFSET + NON_CRITICAL_FRAME_SIZE;

/// Bytes the frames of this chip need, rounded up to the alignment of the buffer.
pub(crate) const BUFFER_SIZE: usize =
    (DEVICE_REGIONS_OFFSET + DEVICE_REGION_WORDS * 4).next_multiple_of(16);
