use esp_radio_rtos_driver::semaphore::SemaphoreKind;
use esp_wifi_sys::c_types::c_void;

use crate::preempt::semaphore::{SemaphoreHandle, SemaphorePtr};

pub(crate) fn mutex_create(recursive: bool) -> *mut c_void {
    let ptr = SemaphoreHandle::new(if recursive {
        SemaphoreKind::RecursiveMutex
    } else {
        SemaphoreKind::Mutex
    })
    .leak()
    .as_ptr()
    .cast();

    trace!("mutex_create -> {:?}", ptr);

    ptr
}

pub(crate) fn mutex_delete(mutex: *mut c_void) {
    let ptr = unwrap!(SemaphorePtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { SemaphoreHandle::from_ptr(ptr) };
    core::mem::drop(handle);
}

pub(crate) fn mutex_lock(mutex: *mut c_void) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };

    handle.take(None) as i32
}

pub(crate) fn mutex_unlock(mutex: *mut c_void) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(mutex.cast()), "mutex is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };

    handle.give() as i32
}
