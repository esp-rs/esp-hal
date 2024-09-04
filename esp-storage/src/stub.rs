use core::{ptr, slice};

use crate::maybe_with_critical_section;

const SUCCESS_CODE: i32 = 0;
const ERROR_CODE: i32 = 1;
const ERASE_BYTE: u8 = 0xff;
const WORD_SIZE: u32 = 4;
const SECTOR_SIZE: u32 = 4 << 10;
const NUM_SECTORS: u32 = 4;
const FLASH_SIZE: u32 = SECTOR_SIZE * NUM_SECTORS;

static mut FLASH_LOCK: bool = true;
static mut FLASH_DATA: [u8; FLASH_SIZE as usize] = [0u8; FLASH_SIZE as usize];

macro_rules! print_error {
    ($($tt:tt)*) => {
        #[cfg(all(test, feature = "emulation"))]
        eprintln!($($tt)*)
    };
}

fn check<const ALIGN: u32, const SIZE: u32, const MAX_LEN: u32>(
    offset: u32,
    length: u32,
    data: *const u32,
) -> bool {
    if offset % ALIGN > 0 {
        print_error!("Not aligned offset: {offset}");
        return false;
    }
    if length % ALIGN > 0 {
        print_error!("Not aligned length: {length}");
        return false;
    }
    if offset > SIZE {
        print_error!("Offset out of range: {offset} > {SIZE}");
        return false;
    }
    if offset + length > SIZE {
        print_error!("Length out of range: {offset} + {length} > {SIZE}");
        return false;
    }
    if length > MAX_LEN {
        print_error!("Length out of range: {length} > {MAX_LEN}");
        return false;
    }
    let addr = unsafe { (data as *const u8).offset_from(ptr::null()) } as u32;
    if addr % ALIGN > 0 {
        print_error!("Not aligned data: {addr:#0x}");
        return false;
    }
    if unsafe { FLASH_LOCK } {
        print_error!("Flash locked");
        return false;
    }
    true
}

pub(crate) fn spiflash_read(src_addr: u32, data: *mut u32, len: u32) -> i32 {
    if check::<WORD_SIZE, FLASH_SIZE, SECTOR_SIZE>(src_addr, len, data) {
        maybe_with_critical_section(|| {
            let src = unsafe { slice::from_raw_parts_mut(data as *mut u8, len as _) };
            unsafe { src.copy_from_slice(&FLASH_DATA[src_addr as usize..][..len as usize]) };
        });
        SUCCESS_CODE
    } else {
        ERROR_CODE
    }
}

pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(|| {
        unsafe { FLASH_LOCK = false };
    });
    SUCCESS_CODE
}

pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    if check::<1, NUM_SECTORS, 1>(sector_number, 1, ptr::null()) {
        maybe_with_critical_section(|| {
            let dst_addr = sector_number * SECTOR_SIZE;
            let len = SECTOR_SIZE;
            unsafe { FLASH_DATA[dst_addr as usize..][..len as usize].fill(ERASE_BYTE) };
        });
        SUCCESS_CODE
    } else {
        ERROR_CODE
    }
}

pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    if check::<WORD_SIZE, FLASH_SIZE, SECTOR_SIZE>(dest_addr, len, data) {
        maybe_with_critical_section(|| {
            let dst = unsafe { slice::from_raw_parts(data as *const u8, len as _) };
            for (d, s) in unsafe { &mut FLASH_DATA[dest_addr as usize..][..len as usize] }
                .iter_mut()
                .zip(dst)
            {
                *d &= *s;
            }
        });
        SUCCESS_CODE
    } else {
        ERROR_CODE
    }
}
