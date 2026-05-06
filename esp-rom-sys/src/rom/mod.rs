//! # Wrappers for selected ROM functions
//!
//! ## Overview
//!
//! For some selected ROM functions safe wrappers are provided for convenience.

#[cfg(any(rom_crc_be, rom_crc_le))]
pub mod crc;
#[cfg(any(rom_md5_bsd, rom_md5_mbedtls))]
pub mod md5;
pub mod spiflash;

/// Busy-loop CPU for the given amount of microseconds.
#[inline(always)]
pub fn ets_delay_us(us: u32) {
    unsafe extern "C" {
        fn ets_delay_us(us: u32);
    }

    unsafe { ets_delay_us(us) };
}

/// Set the real CPU ticks per us to the ets, so that ets_delay_us will be
/// accurate. Call this function when CPU frequency is changed.
#[inline(always)]
pub fn ets_update_cpu_frequency_rom(ticks_per_us: u32) {
    unsafe extern "C" {
        fn ets_update_cpu_frequency(ticks_per_us: u32);
    }

    unsafe { ets_update_cpu_frequency(ticks_per_us) };
}

/// Get the reset reason for CPU.
#[inline(always)]
pub fn rtc_get_reset_reason(cpu_num: u32) -> u32 {
    unsafe extern "C" {
        fn rtc_get_reset_reason(cpu_num: u32) -> u32;
    }

    unsafe { rtc_get_reset_reason(cpu_num) }
}

/// Software Reset digital core.
#[inline(always)]
pub fn software_reset_cpu(cpu_num: u32) {
    unsafe extern "C" {
        fn software_reset_cpu(cpu_num: u32);
    }

    unsafe { software_reset_cpu(cpu_num) };
}

/// Software Reset digital core.
#[inline(always)]
pub fn software_reset() -> ! {
    unsafe extern "C" {
        fn software_reset() -> !;
    }

    unsafe { software_reset() }
}

/// Set App cpu Entry code, code can be called in PRO CPU.
#[cfg(any(esp32s3, esp32p4))]
#[inline(always)]
pub fn ets_set_appcpu_boot_addr(boot_addr: u32) {
    unsafe extern "C" {
        fn ets_set_appcpu_boot_addr(boot_addr: u32);
    }

    unsafe { ets_set_appcpu_boot_addr(boot_addr) };
}

// libphy.a can pull this in on some chips, we provide it here
// so that either ieee or esp-radio gets it for free without duplicating in both
#[unsafe(no_mangle)]
extern "C" fn rtc_clk_xtal_freq_get() -> i32 {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32c6, esp32h2, esp32p4))] {
            unsafe extern "C" {
                fn ets_clk_get_xtal_freq() -> i32;
            }
            (unsafe { ets_clk_get_xtal_freq() }) / 1_000_000
        } else if #[cfg(any(esp32s2, esp32s3, esp32c3))] {
            unsafe extern "C" {
                fn ets_get_xtal_freq() -> i32;
            }
            (unsafe { ets_get_xtal_freq() }) / 1_000_000
        } else if #[cfg(any(esp32, esp32c2))] {
            // just rely on RTC_CNTL_STORE4
            (regs!(RTC_CNTL).store4().read().bits() & 0xff) as i32
        } else if #[cfg(any(esp32c5, esp32c61))]  {
            // PCR_CLK_XTAL_FREQ updates its value based on EFUSE_XTAL_48M_SEL.
            regs!(PCR).sysclk_conf().read().clk_xtal_freq().bits() as i32
        } else {
            compile_error!("rtc_clk_xtal_freq_get not implemented for this chip");
        }
    }
}
