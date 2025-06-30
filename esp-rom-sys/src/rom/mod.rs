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

/// Busy-loop CPU for the given about of us.
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
