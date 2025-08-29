use portable_atomic::{AtomicU32, Ordering};

use crate::{
    binary::include::*,
    hal::{
        peripherals::{LPWR, SYSCON},
        ram,
    },
};

#[unsafe(no_mangle)]
unsafe extern "C" fn abort() {
    trace!("misc_nvs_deinit")
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_deinit() {
    trace!("misc_nvs_deinit")
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_init() -> i32 {
    trace!("misc_nvs_init");
    0
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_restore() -> i32 {
    todo!("misc_nvs_restore")
}

#[unsafe(no_mangle)]
static mut g_log_mod: i32 = 0;

#[unsafe(no_mangle)]
static mut g_log_level: i32 = 0;

#[unsafe(no_mangle)]
pub static mut g_misc_nvs: &u32 = unsafe { &*core::ptr::addr_of!(NVS) };

pub static mut NVS: u32 = 0;
