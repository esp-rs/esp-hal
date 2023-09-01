use crate::{
    binary::include::{esp_timer_create_args_t, esp_timer_handle_t},
    memory_fence::memory_fence,
};
use crate::{debug, panic, trace, unwrap};

static ESP_FAKE_TIMER: () = ();

#[derive(Debug, Clone, Copy)]
pub struct Timer {
    pub ptimer: *mut crate::binary::c_types::c_void,
    pub expire: u64,
    pub period: u64,
    pub active: bool,
    pub timer_ptr: *mut crate::binary::c_types::c_void,
    pub arg_ptr: *mut crate::binary::c_types::c_void,
}

pub static mut TIMERS: [Option<Timer>; 20] = [None; 20];

pub fn compat_timer_arm(ptimer: *mut crate::binary::c_types::c_void, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ptimer, tmout * 1000, repeat);
}

pub fn compat_timer_arm_us(ptimer: *mut crate::binary::c_types::c_void, us: u32, repeat: bool) {
    debug!(
        "timer_arm_us, current time {}",
        crate::timer::get_systimer_count()
    );

    let ticks = us as u64 * (crate::timer::TICKS_PER_SECOND / 1_000_000);
    debug!("timer_arm_us {:?} {} {}", ptimer, ticks, repeat);
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            if let Some(mut timer) = TIMERS[i] {
                memory_fence();

                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    timer.expire = ticks as u64 + crate::timer::get_systimer_count();
                    timer.active = true;
                    if repeat {
                        timer.period = ticks as u64;
                    } else {
                        timer.period = 0;
                    }
                    TIMERS[i] = Some(timer);
                    break;
                }
            }
        }
    });
}

pub fn compat_timer_disarm(ptimer: *mut crate::binary::c_types::c_void) {
    debug!("timer_disarm {:?}", ptimer);
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            memory_fence();
            if let Some(mut timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    timer.active = false;
                    TIMERS[i] = Some(timer);
                    break;
                }
            }
        }
    });
}

pub fn compat_timer_done(ptimer: *mut crate::binary::c_types::c_void) {
    debug!("timer_done {:?}", ptimer);
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            memory_fence();

            if let Some(timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    TIMERS[i] = Some(Timer {
                        active: false,
                        ..timer
                    });

                    let ets_timer = ptimer as *mut crate::binary::include::ets_timer;
                    (*ets_timer).priv_ = core::ptr::null_mut();
                    (*ets_timer).expire = 0;
                    break;
                }
            }
        }
    });
}

pub fn compat_timer_setfn(
    ptimer: *mut crate::binary::c_types::c_void,
    pfunction: *mut crate::binary::c_types::c_void,
    parg: *mut crate::binary::c_types::c_void,
) {
    trace!("timer_setfn {:?} {:?} {:?}", ptimer, pfunction, parg,);

    critical_section::with(|_| unsafe {
        memory_fence();
        let mut success = false;
        for i in 0..TIMERS.len() {
            memory_fence();

            if let Some(timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    TIMERS[i] = Some(Timer {
                        timer_ptr: pfunction,
                        arg_ptr: parg,
                        active: false,
                        ..timer
                    });
                    success = true;
                    break;
                }
            }
        }

        let ets_timer = ptimer as *mut crate::binary::include::ets_timer;
        (*ets_timer).expire = 0;

        if !success {
            for i in 0..TIMERS.len() {
                memory_fence();

                (*ets_timer).next = core::ptr::null_mut();
                (*ets_timer).expire = 0;
                (*ets_timer).period = 0;
                (*ets_timer).func = None;
                (*ets_timer).priv_ = core::ptr::null_mut();

                if TIMERS[i].is_none() {
                    TIMERS[i] = Some(Timer {
                        ptimer: ptimer,
                        expire: 0,
                        period: 0,
                        active: false,
                        timer_ptr: pfunction,
                        arg_ptr: parg,
                    });
                    break;
                }
            }
        }
    });
}

pub fn compat_esp_timer_create(
    args: *const esp_timer_create_args_t,
    mut out_handle: *mut esp_timer_handle_t,
) -> i32 {
    unsafe {
        debug!(
            "esp_timer_create {:?} {:?} {:?}",
            (*args).callback,
            (*args).arg,
            out_handle
        );
    }

    let args = args as *const esp_timer_create_args_t;

    critical_section::with(|_| unsafe {
        let mut success = false;
        memory_fence();

        for i in 0..TIMERS.len() {
            memory_fence();

            debug!("esp_timer_create {}", i);
            if TIMERS[i].is_none() {
                TIMERS[i] = Some(Timer {
                    ptimer: &ESP_FAKE_TIMER as *const _ as *mut crate::binary::c_types::c_void,
                    expire: 0,
                    period: 0,
                    active: false,
                    timer_ptr: core::mem::transmute(unwrap!((*args).callback)),
                    arg_ptr: (*args).arg,
                });
                out_handle = &ESP_FAKE_TIMER as *const _ as *mut esp_timer_handle_t;
                success = true;
                debug!("esp_timer_create {:?} {:?}", args, out_handle);

                break;
            }
        }
        if !success {
            panic!("ran out of timers");
        }
    });

    0
}
