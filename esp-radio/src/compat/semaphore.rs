use esp_wifi_sys::c_types::c_void;

use crate::{
    compat::OSI_FUNCS_TIME_BLOCKING,
    preempt::semaphore::{SemaphoreHandle, SemaphorePtr},
};

pub(crate) fn sem_create(max: u32, init: u32) -> *mut c_void {
    SemaphoreHandle::new(max, init).leak().as_ptr().cast()
}

pub(crate) fn sem_delete(semphr: *mut c_void) {
    let ptr = unwrap!(SemaphorePtr::new(semphr.cast()), "semphr is null");

    let handle = unsafe { SemaphoreHandle::from_ptr(ptr) };
    core::mem::drop(handle);
}

pub(crate) fn sem_take(semphr: *mut c_void, tick: u32) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(semphr.cast()), "semphr is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };
    // Assuming `tick` is in microseconds
    let timeout = if tick == OSI_FUNCS_TIME_BLOCKING {
        None
    } else {
        Some(tick)
    };

    handle.take(timeout) as i32
}

pub(crate) fn sem_take_from_isr(semphr: *mut c_void, _higher_prio_task_waken: *mut bool) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(semphr.cast()), "semphr is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };

    handle.try_take() as i32
}

pub(crate) fn sem_give(semphr: *mut c_void) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(semphr.cast()), "semphr is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };

    handle.give() as i32
}

pub(crate) fn sem_give_from_isr(semphr: *mut c_void, _higher_prio_task_waken: *mut bool) -> i32 {
    let ptr = unwrap!(SemaphorePtr::new(semphr.cast()), "semphr is null");

    let handle = unsafe { SemaphoreHandle::ref_from_ptr(&ptr) };

    handle.give() as i32
}
