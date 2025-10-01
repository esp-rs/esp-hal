use super::*;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct syscall_stub_table {
    pub __getreent: ::core::option::Option<unsafe extern "C" fn() -> *mut _reent>,
    pub _malloc_r: ::core::option::Option<
        unsafe extern "C" fn(r: *mut _reent, arg1: usize) -> *mut ::core::ffi::c_void,
    >,
    pub _free_r: ::core::option::Option<
        unsafe extern "C" fn(r: *mut _reent, arg1: *mut ::core::ffi::c_void),
    >,
    pub _realloc_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *mut ::core::ffi::c_void,
            arg2: usize,
        ) -> *mut ::core::ffi::c_void,
    >,
    pub _calloc_r: ::core::option::Option<
        unsafe extern "C" fn(r: *mut _reent, arg1: usize, arg2: usize) -> *mut ::core::ffi::c_void,
    >,
    pub _abort: ::core::option::Option<unsafe extern "C" fn()>,
    pub _system_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int,
    >,
    pub _rename_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
            arg2: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int,
    >,
    pub _times_r:
        ::core::option::Option<unsafe extern "C" fn(r: *mut _reent, arg1: *mut tms) -> clock_t>,
    pub _gettimeofday_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *mut timeval,
            arg2: *mut ::core::ffi::c_void,
        ) -> ::core::ffi::c_int,
    >,
    pub _raise_r: ::core::option::Option<unsafe extern "C" fn(r: *mut _reent)>,
    pub _unlink_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int,
    >,
    pub _link_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
            arg2: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int,
    >,
    pub _stat_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
            arg2: *mut stat,
        ) -> ::core::ffi::c_int,
    >,
    pub _fstat_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: ::core::ffi::c_int,
            arg2: *mut stat,
        ) -> ::core::ffi::c_int,
    >,
    pub _sbrk_r: ::core::option::Option<
        unsafe extern "C" fn(r: *mut _reent, arg1: isize) -> *mut ::core::ffi::c_void,
    >,
    pub _getpid_r:
        ::core::option::Option<unsafe extern "C" fn(r: *mut _reent) -> ::core::ffi::c_int>,
    pub _kill_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: ::core::ffi::c_int,
            arg2: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub _exit_r:
        ::core::option::Option<unsafe extern "C" fn(r: *mut _reent, arg1: ::core::ffi::c_int)>,
    pub _close_r: ::core::option::Option<
        unsafe extern "C" fn(r: *mut _reent, arg1: ::core::ffi::c_int) -> ::core::ffi::c_int,
    >,
    pub _open_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: *const ::core::ffi::c_char,
            arg2: ::core::ffi::c_int,
            arg3: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub _write_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: ::core::ffi::c_int,
            arg2: *const ::core::ffi::c_void,
            arg3: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub _lseek_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: ::core::ffi::c_int,
            arg2: ::core::ffi::c_int,
            arg3: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub _read_r: ::core::option::Option<
        unsafe extern "C" fn(
            r: *mut _reent,
            arg1: ::core::ffi::c_int,
            arg2: *mut ::core::ffi::c_void,
            arg3: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub _retarget_lock_init: ::core::option::Option<unsafe extern "C" fn(lock: *mut _LOCK_T)>,
    pub _retarget_lock_init_recursive:
        ::core::option::Option<unsafe extern "C" fn(lock: *mut _LOCK_T)>,
    pub _retarget_lock_close: ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    pub _retarget_lock_close_recursive: ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    pub _retarget_lock_acquire: ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    pub _retarget_lock_acquire_recursive:
        ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    pub _retarget_lock_try_acquire:
        ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T) -> ::core::ffi::c_int>,
    pub _retarget_lock_try_acquire_recursive:
        ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T) -> ::core::ffi::c_int>,
    pub _retarget_lock_release: ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    pub _retarget_lock_release_recursive:
        ::core::option::Option<unsafe extern "C" fn(lock: _LOCK_T)>,
    #[allow(clippy::type_complexity)]
    pub _printf_float: ::core::option::Option<
        unsafe extern "C" fn(
            data: *mut _reent,
            pdata: *mut ::core::ffi::c_void,
            fp: *mut FILE,
            pfunc: ::core::option::Option<
                unsafe extern "C" fn(
                    arg1: *mut _reent,
                    arg2: *mut FILE,
                    arg3: *const ::core::ffi::c_char,
                    len: usize,
                ) -> ::core::ffi::c_int,
            >,
            ap: *mut va_list,
        ) -> ::core::ffi::c_int,
    >,
    pub _scanf_float: ::core::option::Option<
        unsafe extern "C" fn(
            rptr: *mut _reent,
            pdata: *mut ::core::ffi::c_void,
            fp: *mut FILE,
            ap: *mut va_list,
        ) -> ::core::ffi::c_int,
    >,
    pub __assert_func: ::core::option::Option<
        unsafe extern "C" fn(
            file: *const ::core::ffi::c_char,
            line: ::core::ffi::c_int,
            func: *const ::core::ffi::c_char,
            failedexpr: *const ::core::ffi::c_char,
        ) -> !,
    >,
    pub __sinit: ::core::option::Option<unsafe extern "C" fn(r: *mut _reent)>,
    pub _cleanup_r: ::core::option::Option<unsafe extern "C" fn(r: *mut _reent)>,
}
