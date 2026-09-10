//! ESP32-C6 and ESP32-H2 retention buffer layout and CPU-domain device regions.

use crate::{
    peripherals::{CLINT, EXTMEM, INTPRI, PLIC_MX, PLIC_UX},
    rtc_cntl::cpu_retention::{
        device_regs::DeviceRegion,
        frames::chip::{CRITICAL_FRAME_SIZE, NON_CRITICAL_FRAME_SIZE},
    },
    system::Cpu,
};

/// `INTPRI_CORE0_CPU_INT_ENABLE_REG` through `INTPRI_RND_ECO_LOW_REG`, at offsets 0x0 to 0xb0.
const INTPRI_WORDS: usize = 45;
/// `PLIC_*XINT_ENABLE_REG` through `PLIC_*XINT_CLAIM_REG`, at offsets 0x0 to 0x94.
const PLIC_INT_WORDS: usize = 38;
/// `CLINT_*INT_SIP_REG` through `CLINT_*INT_*TIMECMP_H_REG`, at offsets 0x0 to 0x14.
const CLINT_WORDS: usize = 6;

/// `PLIC_*XINT_CONF_REG`. The PAC blocks stop at the claim register.
const PLIC_CONF_OFFSET: usize = 0x3fc;

const REGION_COUNT: usize = if cfg!(esp32c6) { 10 } else { 9 };

/// The regions that `esp32c6/sleep_cpu_dynamic.c:45-78` declares.
#[crate::ram]
pub(crate) fn regions() -> [DeviceRegion; REGION_COUNT] {
    let intpri = INTPRI::regs();
    let extmem = EXTMEM::regs();
    let plic_mx = PLIC_MX::regs().mxint_enable().as_ptr();
    let plic_ux = PLIC_UX::regs().uxint_enable().as_ptr();
    let clint = CLINT::regs();

    [
        DeviceRegion::new(intpri.cpu_int_enable().as_ptr(), INTPRI_WORDS),
        DeviceRegion::new(intpri.rnd_eco_high().as_ptr(), 1),
        #[cfg(esp32c6)]
        DeviceRegion::new(extmem.l1_cache_ctrl().as_ptr(), 1),
        DeviceRegion::new(extmem.l1_cache_wrap_around_ctrl().as_ptr(), 1),
        DeviceRegion::new(plic_mx, PLIC_INT_WORDS),
        DeviceRegion::new(plic_mx.wrapping_byte_add(PLIC_CONF_OFFSET), 1),
        DeviceRegion::new(plic_ux, PLIC_INT_WORDS),
        DeviceRegion::new(plic_ux.wrapping_byte_add(PLIC_CONF_OFFSET), 1),
        DeviceRegion::new(clint.msip().as_ptr(), CLINT_WORDS),
        DeviceRegion::new(clint.usip().as_ptr(), CLINT_WORDS),
    ]
}

pub(crate) const DEVICE_REGION_WORDS: usize =
    INTPRI_WORDS + 1 + 1 + cfg!(esp32c6) as usize + 2 * (PLIC_INT_WORDS + 1) + 2 * CLINT_WORDS;

pub(crate) const CRITICAL_FRAME_OFFSET: usize = 0;
pub(crate) const NON_CRITICAL_FRAME_OFFSET: usize = CRITICAL_FRAME_OFFSET + CRITICAL_FRAME_SIZE;
pub(crate) const DEVICE_REGIONS_OFFSET: usize = NON_CRITICAL_FRAME_OFFSET + NON_CRITICAL_FRAME_SIZE;

/// Bytes one core's frames need, rounded up to the alignment of the buffer.
pub(crate) const BLOCK_SIZE: usize =
    (DEVICE_REGIONS_OFFSET + DEVICE_REGION_WORDS * 4).next_multiple_of(16);

/// Bytes every core's frames need.
pub(crate) const BUFFER_SIZE: usize = BLOCK_SIZE * Cpu::COUNT;

/// `RTC_SLEEP_WAKE_STUB_ADDR_REG`: the word that holds the wake stub address across the sleep.
#[crate::ram]
pub(crate) fn wake_stub_reg() -> *mut u32 {
    crate::peripherals::LP_AON::regs().store8().as_ptr()
}
