#![allow(non_camel_case_types)]

use core::ffi::{c_char, c_int, c_long, c_void};

// future chips or ECOs _might_ be different - at least ESP-IDF defines the struct per chip
#[cfg_attr(
    any(esp32, esp32s2, esp32s3, esp32c2, esp32c3, esp32c6, esp32h2),
    path = "v1.rs"
)]
pub(crate) mod chip_specific;

pub type clock_t = c_long;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct tms {
    _unused: [u8; 0],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct timeval {
    _unused: [u8; 0],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct stat {
    _unused: [u8; 0],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct __lock {
    _unused: [u8; 0],
}

pub type _LOCK_T = *mut __lock;

pub type _lock_t = _LOCK_T;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct _iobuf {
    pub _placeholder: *mut c_void,
}

#[allow(clippy::upper_case_acronyms)]
pub type __FILE = _iobuf;

#[allow(clippy::upper_case_acronyms)]
pub type FILE = __FILE;

pub type __tm = ();
pub type _rand48 = ();
pub type _misc_reent = ();
pub type __locale_t = ();
pub type _mprec = ();
pub type _on_exit_args = ();

#[repr(C)]
#[derive(Copy, Clone)]
pub struct _atexit {
    pub _next: *mut _atexit,
    pub _ind: c_int,
    pub _fns: [Option<unsafe extern "C" fn()>; 32],
    pub _on_exit_args_ptr: *mut _on_exit_args,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct _glue {
    pub _next: *mut _glue,
    pub _niobs: c_int,
    pub _iobs: *mut __FILE,
}

/// Reentrancy struct.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct _reent {
    // Consider as opaque for now.
    _errno: c_int,
    _stdin: *mut __FILE,
    _stdout: *mut __FILE,
    _stderr: *mut __FILE,
    _inc: c_int,
    _emergency: *mut c_char,
    _reserved_0: c_int,
    _reserved_1: c_int,
    _locale: *mut __locale_t,
    _mp: *mut _mprec,
    __cleanup: Option<unsafe extern "C" fn(arg1: *mut _reent)>,
    _gamma_signgam: c_int,
    _cvtlen: c_int,
    _cvtbuf: *mut c_char,
    _r48: *mut _rand48,
    _localtime_buf: *mut __tm,
    _asctime_buf: *mut c_char,
    _sig_func: *mut Option<unsafe extern "C" fn(arg1: c_int)>,
    _reserved_6: *mut _atexit,
    _reserved_7: _atexit,
    _reserved_8: _glue,
    __sf: *mut __FILE,
    _misc: *mut _misc_reent,
    _signal_buf: *mut c_char,
}

pub type va_list = *mut ::core::ffi::c_char;

/// The syscall table that ROM functions use.
#[allow(clippy::missing_transmute_annotations)]
pub static mut SYSCALL_TABLE: chip_specific::syscall_stub_table = unsafe {
    chip_specific::syscall_stub_table {
        __getreent: Some(core::mem::transmute(not_implemented as *const fn())),
        _malloc_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _free_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _realloc_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _calloc_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _abort: Some(abort_wrapper),
        _system_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _rename_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _times_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _gettimeofday_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _raise_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _unlink_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _link_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _stat_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _fstat_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _sbrk_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _getpid_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _kill_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _exit_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _close_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _open_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _write_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _lseek_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _read_r: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_init: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_init_recursive: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_close: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_close_recursive: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_acquire: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_acquire_recursive: Some(core::mem::transmute(
            not_implemented as *const fn(),
        )),
        _retarget_lock_try_acquire: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_try_acquire_recursive: Some(core::mem::transmute(
            not_implemented as *const fn(),
        )),
        _retarget_lock_release: Some(core::mem::transmute(not_implemented as *const fn())),
        _retarget_lock_release_recursive: Some(core::mem::transmute(
            not_implemented as *const fn(),
        )),
        _printf_float: Some(core::mem::transmute(not_implemented as *const fn())),
        _scanf_float: Some(core::mem::transmute(not_implemented as *const fn())),
        __assert_func: Some(core::mem::transmute(not_implemented as *const fn())),
        __sinit: Some(core::mem::transmute(not_implemented as *const fn())),
        _cleanup_r: Some(core::mem::transmute(not_implemented as *const fn())),
    }
};

unsafe extern "C" fn not_implemented() {
    panic!("Function called via syscall table is not implemented!");
}

unsafe extern "C" fn abort_wrapper() {
    panic!("Abort called from ROM code!");
}

/// Initialize the syscall table.
///
/// # Safety
/// Should only get called once.
#[allow(clippy::missing_transmute_annotations)]
pub unsafe fn init_syscall_table() {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            unsafe extern "C" {
                static mut syscall_table_ptr_pro: *const chip_specific::syscall_stub_table;
                static mut syscall_table_ptr_app: *const chip_specific::syscall_stub_table;
            }
            unsafe {
                syscall_table_ptr_pro = &raw const SYSCALL_TABLE;
                syscall_table_ptr_app = &raw const SYSCALL_TABLE;
            }
        } else if #[cfg(esp32s2)] {
            unsafe extern "C" {
                static mut syscall_table_ptr_pro: *const chip_specific::syscall_stub_table;
            }
            unsafe { syscall_table_ptr_pro = &raw const SYSCALL_TABLE; }
        } else {
            unsafe extern "C" {
                static mut syscall_table_ptr: *const chip_specific::syscall_stub_table;
            }
            unsafe { syscall_table_ptr = &raw const SYSCALL_TABLE; }
        }
    };
}
