//! # ESP ROM libraries
//!
//! ## Overview
//! The `rom` driver provides functionality related to the ROM (Read-Only
//! Memory) on ESP chips. It includes implementations for the [CRC (Cyclic
//! Redundancy Check)] and [MD5 (Message Digest 5)] algorithms.
//!
//! The driver's functionality allows users to perform CRC calculations and MD5
//! hashing using the ROM functions provided by the ESP chip. This can be useful
//! for various applications that require data integrity checks or cryptographic
//! operations.
//!
//! It uses `CRC` error-checking techniques to detect changes in data during
//! transmission or storage.
//!
//! This module also implements the `MD5` algorithm, which is widely used for
//! cryptographic hash function. It's commonly used to verify data integrity and
//! to check whether the data has been modified.
//!
//! Safe abstractions to the additional libraries provided in the ESP's
//! Read-Only Memory.
//!
//! [CRC (Cyclic Redundancy Check)]: ./crc/index.html
//! [MD5 (Message Digest 5)]: ./md5/index.html

#![allow(unused_macros)]

#[cfg(any(rom_crc_be, rom_crc_le))]
pub mod crc;
#[cfg(any(rom_md5_bsd, rom_md5_mbedtls))]
pub mod md5;

#[allow(unused)]
extern "C" {
    pub(crate) fn rom_i2c_writeReg(block: u32, block_hostid: u32, reg_add: u32, indata: u32);

    pub(crate) fn rom_i2c_writeReg_Mask(
        block: u32,
        block_hostid: u32,
        reg_add: u32,
        reg_add_msb: u32,
        reg_add_lsb: u32,
        indata: u32,
    );
}

macro_rules! regi2c_write {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste::paste! {
            #[allow(unused_unsafe)]
            unsafe {
                $crate::rom::rom_i2c_writeReg(
                    $block as u32,
                    [<$block _HOSTID>] as u32,
                    $reg_add as u32,
                    $indata as u32
                )
            }
        }
    };
}

#[allow(unused_imports)]
pub(crate) use regi2c_write;

macro_rules! regi2c_write_mask {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste::paste! {
            #[allow(unused_unsafe)]
            unsafe {
                $crate::rom::rom_i2c_writeReg_Mask(
                    $block as u32,
                    [<$block _HOSTID>] as u32,
                    $reg_add as u32,
                    [<$reg_add _MSB>] as u32,
                    [<$reg_add _LSB>] as u32,
                    $indata as u32
                )
            }
        }
    };
}

#[allow(unused_imports)]
pub(crate) use regi2c_write_mask;

#[inline(always)]
pub(crate) fn ets_delay_us(us: u32) {
    extern "C" {
        fn ets_delay_us(us: u32);
    }

    unsafe { ets_delay_us(us) };
}

#[allow(unused)]
#[inline(always)]
pub(crate) fn ets_update_cpu_frequency_rom(ticks_per_us: u32) {
    extern "C" {
        fn ets_update_cpu_frequency(ticks_per_us: u32);
    }

    unsafe { ets_update_cpu_frequency(ticks_per_us) };
}

#[inline(always)]
pub(crate) fn rtc_get_reset_reason(cpu_num: u32) -> u32 {
    extern "C" {
        fn rtc_get_reset_reason(cpu_num: u32) -> u32;
    }

    unsafe { rtc_get_reset_reason(cpu_num) }
}

#[inline(always)]
pub(crate) fn software_reset_cpu() {
    extern "C" {
        fn software_reset_cpu();
    }

    unsafe { software_reset_cpu() };
}

#[inline(always)]
pub(crate) fn software_reset() {
    extern "C" {
        fn software_reset();
    }

    unsafe { software_reset() };
}

#[cfg(esp32s3)]
#[inline(always)]
pub(crate) fn ets_set_appcpu_boot_addr(boot_addr: u32) {
    extern "C" {
        fn ets_set_appcpu_boot_addr(boot_addr: u32);
    }

    unsafe { ets_set_appcpu_boot_addr(boot_addr) };
}
