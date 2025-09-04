use esp_wifi_sys::c_types::c_void;

use crate::{
    compat::OSI_FUNCS_TIME_BLOCKING,
    preempt::mutex::{MutexHandle, MutexPtr},
};

pub(crate) fn mutex_create(recursive: bool) -> *mut c_void {
    MutexHandle::new(recursive).leak().as_ptr().cast()
}

pub(crate) fn mutex_delete(mutex: *mut c_void) {
    let ptr = unwrap!(MutexPtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { MutexHandle::from_ptr(ptr) };
    core::mem::drop(handle);
}

pub(crate) fn mutex_lock(mutex: *mut c_void, tick: u32) -> i32 {
    let ptr = unwrap!(MutexPtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { MutexHandle::ref_from_ptr(&ptr) };
    // Assuming `tick` is in microseconds
    let timeout = if tick == OSI_FUNCS_TIME_BLOCKING {
        None
    } else {
        Some(tick)
    };

    handle.lock(timeout) as i32
}

pub(crate) fn mutex_unlock(mutex: *mut c_void) -> i32 {
    let ptr = unwrap!(MutexPtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { MutexHandle::ref_from_ptr(&ptr) };

    handle.unlock() as i32
}
