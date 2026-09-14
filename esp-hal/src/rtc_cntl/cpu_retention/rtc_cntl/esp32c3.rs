use core::ptr::NonNull;

use crate::{
    peripherals::{APB_CTRL, LPWR},
    rtc_cntl::{cpu_retention, sleep::RtcSleepConfig},
};

// The cache needs no maintenance around a retained sleep on this chip: the tag memory stays
// powered, so it survives the CPU power-down. This is why the chip has no tag memory retention
// feature, and why `rtc_cntl_hal_enable_cpu_retention` and its disable counterpart touch no cache.

// `SOC_RTC_CNTL_CPU_PD_REG_FILE_NUM` (108) times `SOC_RTC_CNTL_CPU_PD_DMA_BLOCK_SIZE` (16).
const _: () = ::core::assert!(cpu_retention::payload_size() == 108 * 16);

/// The last of the four configuration words that the CPU frames begin with. The S3 writes a
/// different value here, so the word belongs to the chip and not to the shared descriptor code.
const RETENTION_CONFIG_WORD3: u32 = 0xffff_ffff;

/// Couples CPU power-down to the installed retention buffer, for a light sleep.
///
/// The bit is written and not only set, so that a configuration from [`RtcSleepConfig::deep`]
/// cannot carry a power-down into a light sleep that has no retention memory.
pub(crate) fn configure_cpu_retention(config: &mut RtcSleepConfig, buffer: Option<NonNull<u8>>) {
    config.set_cpu_pd_en(buffer.is_some());
}

/// Prepares CPU retention for the upcoming sleep.
pub(crate) fn prepare_cpu_retention(buffer: Option<NonNull<u8>>) {
    let Some(buffer) = buffer else {
        return;
    };

    unsafe {
        cpu_retention::init_cpu_dma_link(buffer, RETENTION_CONFIG_WORD3);
    }

    enable_cpu_retention(buffer.addr().get());
}

/// Finishes CPU retention after the sleep request returns.
pub(crate) fn finish_cpu_retention(buffer: Option<NonNull<u8>>, _rejected: bool) {
    if buffer.is_none() {
        return;
    }

    // Disarm on every exit, including a rejected request that never slept. A stale descriptor
    // would otherwise affect the next unretained sleep.
    disable_cpu_retention();
}

fn enable_cpu_retention(link_addr: usize) {
    // The field is 27 bits and the address needs 30, so the write drops the top three. The DMA
    // reaches internal SRAM only, so the hardware supplies those bits; esp-idf truncates the same
    // way through `REG_SET_FIELD`. `modify` keeps `nobypass_cpu_iso_rst`, which shares the
    // register.
    APB_CTRL::regs()
        .retention_ctrl()
        .modify(|_, w| unsafe { w.retention_link_addr().bits(link_addr as u32) });

    // The retention timing fields keep their reset values of 20, 3 and 2 cycles.
    // `rtc_cntl_hal_enable_cpu_retention` lengthens them to the maximum on the S3, which has five
    // times as many register frames and the cache tag memory to move, and leaves them alone here.

    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.dig_clk8m_en().set_bit());

    LPWR::regs()
        .retention_ctrl()
        .modify(|_, w| w.retention_en().set_bit());
}

fn disable_cpu_retention() {
    LPWR::regs()
        .retention_ctrl()
        .modify(|_, w| w.retention_en().clear_bit());
}
