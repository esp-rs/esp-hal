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
#[cfg(esp32s3)]
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
        if #[cfg(any(esp32c6, esp32h2))] {
            unsafe extern "C" {
                fn ets_clk_get_xtal_freq() -> i32;
            }
            (unsafe { ets_clk_get_xtal_freq() }) / 1_000_000
        } else if #[cfg(any(esp32s2, esp32s3, esp32c3))] {
            unsafe extern "C" {
                fn ets_get_xtal_freq() -> i32;
            }
            (unsafe { ets_get_xtal_freq() }) / 1_000_000
        } else if #[cfg(esp32)] {
            // the ROM function returns something close to 80_000_000
            // just rely on RTC_CNTL_STORE4
            let rtc_cntl_store4: *const u32 = 0x3FF480B0 as *const u32;
            ((unsafe { rtc_cntl_store4.read_volatile() })  & 0xff) as i32
        } else if #[cfg(esp32c2)] {
            // the ROM function returns 40 also for a 26 MHz xtal
            // just rely on RTC_CNTL_STORE4
            let rtc_cntl_store4: *const u32 = 0x600080AC as *const u32;
            ((unsafe { rtc_cntl_store4.read_volatile() }) & 0xff) as i32
        } else {
            compile_error!("rtc_clk_xtal_freq_get not implemented for this chip");
        }
    }
}
