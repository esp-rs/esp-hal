use crate::{
    binary::{
        c_types,
        include::{esp_timer_create_args_t, esp_timer_handle_t, ets_timer},
    },
    memory_fence::memory_fence,
};

static ESP_FAKE_TIMER: () = ();

#[derive(Debug, Clone, Copy)]
pub struct Timer {
    pub ptimer: *mut c_types::c_void,
    pub expire: u64,
    pub period: u64,
    pub active: bool,
    pub timer_ptr: *mut c_types::c_void,
    pub arg_ptr: *mut c_types::c_void,
}

pub static mut TIMERS: [Option<Timer>; 20] = [None; 20];

pub fn compat_timer_arm(ptimer: *mut c_types::c_void, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ptimer, tmout * 1000, repeat);
}

pub fn compat_timer_arm_us(ptimer: *mut c_types::c_void, us: u32, repeat: bool) {
    let systick = crate::timer::get_systimer_count();

    let ticks = crate::timer::micros_to_ticks(us as u64);
    debug!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ptimer as usize, systick, ticks, repeat
    );
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            if let Some(ref mut timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    timer.expire = ticks + systick;
                    timer.active = true;
                    timer.period = if repeat { ticks } else { 0 };
                    break;
                }
            }
        }

        memory_fence();
    });
}

pub fn compat_timer_disarm(ptimer: *mut c_types::c_void) {
    debug!("timer_disarm {:x}", ptimer as usize);
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            if let Some(ref mut timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    timer.active = false;
                    break;
                }
            }
        }

        memory_fence();
    });
}

pub fn compat_timer_done(ptimer: *mut c_types::c_void) {
    debug!("timer_done {:x}", ptimer as usize);
    critical_section::with(|_| unsafe {
        memory_fence();

        for i in 0..TIMERS.len() {
            if let Some(ref mut timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    trace!("found timer ...");
                    timer.active = false;

                    let ets_timer = ptimer as *mut ets_timer;
                    (*ets_timer).priv_ = core::ptr::null_mut();
                    (*ets_timer).expire = 0;
                    break;
                }
            }
        }

        memory_fence();
    });
}

pub fn compat_timer_setfn(
    ptimer: *mut c_types::c_void,
    pfunction: *mut c_types::c_void,
    parg: *mut c_types::c_void,
) {
    debug!(
        "timer_setfn {:x} {:?} {:?}",
        ptimer as usize, pfunction, parg
    );

    critical_section::with(|_| unsafe {
        memory_fence();
        let mut timer_found = false;
        for i in 0..TIMERS.len() {
            if let Some(ref mut timer) = TIMERS[i] {
                if timer.ptimer == ptimer {
                    timer.timer_ptr = pfunction;
                    timer.arg_ptr = parg;
                    timer.active = false;

                    timer_found = true;
                    break;
                }
            }
        }

        let ets_timer = ptimer as *mut ets_timer;
        (*ets_timer).expire = 0;

        if !timer_found {
            (*ets_timer).next = core::ptr::null_mut();
            (*ets_timer).period = 0;
            (*ets_timer).func = None;
            (*ets_timer).priv_ = core::ptr::null_mut();

            for i in 0..TIMERS.len() {
                if TIMERS[i].is_none() {
                    TIMERS[i] = Some(Timer {
                        ptimer,
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
        memory_fence();
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

    critical_section::with(|_| unsafe {
        let mut timer_found = false;
        memory_fence();

        for i in 0..TIMERS.len() {
            if TIMERS[i].is_none() {
                TIMERS[i] = Some(Timer {
                    ptimer: &ESP_FAKE_TIMER as *const _ as *mut c_types::c_void,
                    expire: 0,
                    period: 0,
                    active: false,
                    timer_ptr: core::mem::transmute(unwrap!((*args).callback)),
                    arg_ptr: (*args).arg,
                });
                out_handle = &ESP_FAKE_TIMER as *const _ as *mut esp_timer_handle_t;
                timer_found = true;
                debug!("esp_timer_create {:?} {:?}", args, out_handle);

                break;
            }
        }
        if !timer_found {
            panic!("ran out of timers");
        }
        memory_fence();
    });

    0
}
