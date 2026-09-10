//! ESP32-C5 and ESP32-C61 retention buffer layout and CPU-domain device regions.

use crate::{
    peripherals::{CACHE, CLIC, CLINT},
    rtc_cntl::cpu_retention::{
        device_regs::DeviceRegion,
        frames::chip::{CRITICAL_FRAME_SIZE, NON_CRITICAL_FRAME_SIZE},
    },
    system::Cpu,
};

/// `CACHE_L1_CACHE_AUTOLOAD_CTRL_REG` through `CACHE_L1_CACHE_AUTOLOAD_SCT1_SIZE_REG`.
const CACHE_WORDS: usize = 5;
/// `CLIC_INT_CONFIG_REG` through `CLIC_INT_THRESH_REG`.
const CLIC_CONFIG_WORDS: usize = 3;
/// `CLIC_INT_CTRL_REG(0)` through `CLIC_INT_CTRL_REG(47)`.
const CLIC_CTRL_WORDS: usize = 48;
/// `CLINT_MINT_SIP_REG`.
const CLINT_SIP_WORDS: usize = 1;
/// `CLINT_MINT_MTIMECMP_L_REG` through `CLINT_MINT_MTIMECMP_H_REG`.
const CLINT_TIMECMP_WORDS: usize = 2;
/// `CLINT_MINT_TIMECTL_REG`.
const CLINT_TIMECTL_WORDS: usize = 1;
/// `CLINT_MINT_MTIME_L_REG` through `CLINT_MINT_MTIME_H_REG`.
const CLINT_MTIME_WORDS: usize = 2;

/// The regions that `esp32c5/sleep_cpu_dynamic.c:43-68` declares.
#[crate::ram]
pub(crate) fn regions() -> [DeviceRegion; 7] {
    let cache = CACHE::regs();
    let clic = CLIC::regs();
    let clint = CLINT::regs();

    [
        DeviceRegion::new(cache.cache_autoload_ctrl().as_ptr(), CACHE_WORDS),
        // The PAC stops at `int_info`, so the region reaches `CLIC_INT_THRESH_REG` by length.
        DeviceRegion::new(clic.int_config().as_ptr(), CLIC_CONFIG_WORDS),
        DeviceRegion::new(clic.int_ip(0).as_ptr().cast(), CLIC_CTRL_WORDS),
        DeviceRegion::new(clint.msip().as_ptr(), CLINT_SIP_WORDS),
        DeviceRegion::new(clint.mtimecmp().as_ptr().cast(), CLINT_TIMECMP_WORDS),
        DeviceRegion::new(clint.mtimectl().as_ptr(), CLINT_TIMECTL_WORDS),
        DeviceRegion::new(clint.mtime().as_ptr().cast(), CLINT_MTIME_WORDS),
    ]
}

pub(crate) const DEVICE_REGION_WORDS: usize = CACHE_WORDS
    + CLIC_CONFIG_WORDS
    + CLIC_CTRL_WORDS
    + CLINT_SIP_WORDS
    + CLINT_TIMECMP_WORDS
    + CLINT_TIMECTL_WORDS
    + CLINT_MTIME_WORDS;

const _: () = ::core::assert!(DEVICE_REGION_WORDS == 62);

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
