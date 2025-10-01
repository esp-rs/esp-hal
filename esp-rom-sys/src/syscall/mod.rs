#![allow(non_camel_case_types)]

// future chips or ECOs _might_ be different - at least ESP-IDF defines the struct per chip
#[cfg_attr(
    any(esp32, esp32s2, esp32s3, esp32c2, esp32c3, esp32c6, esp32h2),
    path = "v1.rs"
)]
pub(crate) mod chip_specific;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct _reent {
    _unused: [u8; 0],
    // the real struct is found here: https://sourceware.org/git/?p=newlib-cygwin.git;a=blob;f=newlib/libc/include/sys/reent.h;h=eafac960fd6ca374b7503f50bf8aa2bd1ee1573e;hb=refs/heads/main#l379
}

pub type clock_t = ::core::ffi::c_long;

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
    pub _placeholder: *mut ::core::ffi::c_void,
}

#[allow(clippy::upper_case_acronyms)]
pub type FILE = _iobuf;

pub type va_list = *mut ::core::ffi::c_char;

static mut S_STUB_TABLE: core::mem::MaybeUninit<chip_specific::syscall_stub_table> =
    core::mem::MaybeUninit::uninit();

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
    let syscall_table = unsafe {
        (&mut *core::ptr::addr_of_mut!(S_STUB_TABLE)).write(chip_specific::syscall_stub_table {
            __getreent: Some(core::mem::transmute(not_implemented as usize)),
            _malloc_r: Some(core::mem::transmute(not_implemented as usize)),
            _free_r: Some(core::mem::transmute(not_implemented as usize)),
            _realloc_r: Some(core::mem::transmute(not_implemented as usize)),
            _calloc_r: Some(core::mem::transmute(not_implemented as usize)),
            _abort: Some(abort_wrapper),
            _system_r: Some(core::mem::transmute(not_implemented as usize)),
            _rename_r: Some(core::mem::transmute(not_implemented as usize)),
            _times_r: Some(core::mem::transmute(not_implemented as usize)),
            _gettimeofday_r: Some(core::mem::transmute(not_implemented as usize)),
            _raise_r: Some(core::mem::transmute(not_implemented as usize)),
            _unlink_r: Some(core::mem::transmute(not_implemented as usize)),
            _link_r: Some(core::mem::transmute(not_implemented as usize)),
            _stat_r: Some(core::mem::transmute(not_implemented as usize)),
            _fstat_r: Some(core::mem::transmute(not_implemented as usize)),
            _sbrk_r: Some(core::mem::transmute(not_implemented as usize)),
            _getpid_r: Some(core::mem::transmute(not_implemented as usize)),
            _kill_r: Some(core::mem::transmute(not_implemented as usize)),
            _exit_r: Some(core::mem::transmute(not_implemented as usize)),
            _close_r: Some(core::mem::transmute(not_implemented as usize)),
            _open_r: Some(core::mem::transmute(not_implemented as usize)),
            _write_r: Some(core::mem::transmute(not_implemented as usize)),
            _lseek_r: Some(core::mem::transmute(not_implemented as usize)),
            _read_r: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_init: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_init_recursive: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_close: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_close_recursive: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_acquire: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_acquire_recursive: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_try_acquire: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_try_acquire_recursive: Some(core::mem::transmute(
                not_implemented as usize,
            )),
            _retarget_lock_release: Some(core::mem::transmute(not_implemented as usize)),
            _retarget_lock_release_recursive: Some(core::mem::transmute(not_implemented as usize)),
            _printf_float: Some(core::mem::transmute(not_implemented as usize)),
            _scanf_float: Some(core::mem::transmute(not_implemented as usize)),
            __assert_func: Some(super::__assert_func),
            __sinit: Some(core::mem::transmute(not_implemented as usize)),
            _cleanup_r: Some(core::mem::transmute(not_implemented as usize)),
        })
    };

    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            unsafe extern "C" {
                static mut syscall_table_ptr_pro: *const chip_specific::syscall_stub_table;
                static mut syscall_table_ptr_app: *const chip_specific::syscall_stub_table;
            }
            unsafe {
                syscall_table_ptr_pro = syscall_table;
                syscall_table_ptr_app = syscall_table;
            }
        } else if #[cfg(esp32s2)] {
            unsafe extern "C" {
                static mut syscall_table_ptr_pro: *const chip_specific::syscall_stub_table;
            }
            unsafe { syscall_table_ptr_pro = syscall_table; }
        } else {
            unsafe extern "C" {
                static mut syscall_table_ptr: *const chip_specific::syscall_stub_table;
            }
            unsafe { syscall_table_ptr = syscall_table; }
        }
    };
}
