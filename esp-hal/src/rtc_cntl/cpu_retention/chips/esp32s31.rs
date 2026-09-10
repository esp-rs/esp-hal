//! ESP32-S31 retention buffer layout and CPU-domain device regions.

use crate::{
    peripherals::CLIC,
    rtc_cntl::cpu_retention::{
        device_regs::DeviceRegion,
        frames::chip::{CRITICAL_FRAME_SIZE, NON_CRITICAL_FRAME_SIZE},
    },
    system::Cpu,
};

/// `CLIC_INT_CONFIG_REG`.
const CLIC_CONFIG_WORDS: usize = 1;
/// `CLIC_INT_THRESH_REG`.
const CLIC_THRESH_WORDS: usize = 1;
/// `CLIC_INT_CTRL_REG(0)` through `CLIC_INT_CTRL_REG(47)`. Each word holds the pending, enable,
/// attribute and level bytes of one interrupt, which the PAC describes as four byte registers.
const CLIC_CTRL_WORDS: usize = 48;

/// The regions that `esp32s31/sleep_cpu_dynamic.c:44-49` declares.
#[crate::ram]
pub(crate) fn regions() -> [DeviceRegion; 3] {
    let clic = CLIC::regs();

    [
        DeviceRegion::new(clic.int_config().as_ptr(), CLIC_CONFIG_WORDS),
        DeviceRegion::new(clic.int_thresh().as_ptr(), CLIC_THRESH_WORDS),
        DeviceRegion::new(clic.int_ip(0).as_ptr().cast(), CLIC_CTRL_WORDS),
    ]
}

pub(crate) const DEVICE_REGION_WORDS: usize =
    CLIC_CONFIG_WORDS + CLIC_THRESH_WORDS + CLIC_CTRL_WORDS;

const _: () = ::core::assert!(DEVICE_REGION_WORDS == 50);

pub(crate) const CRITICAL_FRAME_OFFSET: usize = 0;
pub(crate) const NON_CRITICAL_FRAME_OFFSET: usize = CRITICAL_FRAME_OFFSET + CRITICAL_FRAME_SIZE;
pub(crate) const DEVICE_REGIONS_OFFSET: usize = NON_CRITICAL_FRAME_OFFSET + NON_CRITICAL_FRAME_SIZE;

/// Bytes one core's frames need, rounded up to the alignment of the buffer.
pub(crate) const BLOCK_SIZE: usize =
    (DEVICE_REGIONS_OFFSET + DEVICE_REGION_WORDS * 4).next_multiple_of(16);

/// Bytes every core's frames need.
pub(crate) const BUFFER_SIZE: usize = BLOCK_SIZE * Cpu::COUNT;

/// `RTC_SLEEP_WAKE_STUB_ADDR_REG`: the word that holds the wake stub address across the sleep. On
/// this chip it is `LP_SYSTEM_REG_LP_STORE8_REG`, not the `LP_AON` word the other chips use.
#[crate::ram]
pub(crate) fn wake_stub_reg() -> *mut u32 {
    crate::peripherals::LP_SYS::regs().lp_store(8).as_ptr()
}
