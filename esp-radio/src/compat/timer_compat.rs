use esp_radio_rtos_driver::timer::TimerHandle;

use crate::{
    binary::{c_types::c_void, include::ets_timer},
    preempt::timer::TimerPtr,
};

pub(crate) fn compat_timer_arm(ets_timer: *mut ets_timer, tmout_ms: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout_ms.saturating_mul(1000), repeat);
}

pub(crate) fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    trace!(
        "timer_arm_us {:x} current: {} micros: {} repeat: {}",
        ets_timer as usize,
        crate::preempt::now(),
        us,
        repeat
    );

    let ets_timer = unwrap!(unsafe { ets_timer.as_mut() }, "ets_timer is null");

    let timer = unwrap!(TimerPtr::new(ets_timer.priv_.cast()), "timer is null");
    let timer = unsafe { TimerHandle::ref_from_ptr(&timer) };

    timer.arm(us as u64, repeat);
}

pub(crate) fn compat_timer_disarm(ets_timer: *mut ets_timer) {
    trace!("timer disarm");
    let ets_timer = unwrap!(unsafe { ets_timer.as_mut() }, "ets_timer is null");

    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::ref_from_ptr(&timer) };

        timer.disarm();
    }
}

pub(crate) fn compat_timer_is_active(ets_timer: *mut ets_timer) -> bool {
    trace!("timer is_active");
    let ets_timer = unwrap!(unsafe { ets_timer.as_mut() }, "ets_timer is null");

    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::ref_from_ptr(&timer) };

        timer.is_active()
    } else {
        false
    }
}

fn delete_timer(ets_timer: &mut ets_timer) {
    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::from_ptr(timer) };

        core::mem::drop(timer);
        ets_timer.priv_ = core::ptr::null_mut();
    }
}

pub(crate) fn compat_timer_done(ets_timer: *mut ets_timer) {
    trace!("timer done");

    let ets_timer = unwrap!(unsafe { ets_timer.as_mut() }, "ets_timer is null");

    delete_timer(ets_timer);
}

pub(crate) fn compat_timer_setfn(
    ets_timer: *mut ets_timer,
    pfunction: unsafe extern "C" fn(*mut c_void),
    parg: *mut c_void,
) {
    trace!(
        "timer_setfn {:x} {:?} {:?}",
        ets_timer as usize, pfunction, parg
    );

    let ets_timer = unwrap!(unsafe { ets_timer.as_mut() }, "ets_timer is null");

    // This function is expected to create timers. For the simplicity of the preempt API, we
    // will not update existing timers, but create new ones.
    delete_timer(ets_timer);

    let timer = unsafe { TimerHandle::new(pfunction, parg) }
        .leak()
        .cast()
        .as_ptr();

    ets_timer.next = core::ptr::null_mut();
    ets_timer.period = 0;
    ets_timer.func = None;
    ets_timer.priv_ = timer;
}
