use crate::binary::{
    c_types,
    include::{esp_timer_create_args_t, esp_timer_handle_t, ets_timer},
};

#[derive(Clone, Copy, Debug)]
pub(crate) struct TimerCallback {
    f: unsafe extern "C" fn(*mut c_types::c_void),
    args: *mut c_types::c_void,
}

impl TimerCallback {
    fn new(f: unsafe extern "C" fn(*mut c_types::c_void), args: *mut c_types::c_void) -> Self {
        Self { f, args }
    }

    pub fn call(self) {
        unsafe { (self.f)(self.args) };
    }
}

impl From<&esp_timer_create_args_t> for TimerCallback {
    fn from(args: &esp_timer_create_args_t) -> Self {
        Self::new(unwrap!(args.callback), args.arg)
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Timer {
    pub ets_timer: *mut ets_timer,
    pub started: u64,
    pub timeout: u64,
    pub active: bool,
    pub periodic: bool,
    pub callback: TimerCallback,
}

impl Timer {
    pub fn id(&self) -> usize {
        self.ets_timer as usize
    }
}
#[cfg(coex)]
pub(crate) static mut TIMERS: heapless::Vec<Timer, 34> = heapless::Vec::new();
#[cfg(not(coex))]
pub(crate) static mut TIMERS: heapless::Vec<Timer, 20> = heapless::Vec::new();

pub fn compat_timer_arm(ets_timer: *mut ets_timer, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout * 1000, repeat);
}

pub fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    let systick = crate::timer::get_systimer_count();
    let ticks = crate::timer::micros_to_ticks(us as u64);

    debug!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ets_timer as usize, systick, ticks, repeat
    );

    critical_section::with(|_| unsafe {
        if let Some(timer) = TIMERS.iter_mut().find(|t| t.ets_timer == ets_timer) {
            timer.started = systick;
            timer.timeout = ticks;
            timer.active = true;
            timer.periodic = repeat;
        } else {
            debug!("timer_arm_us {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_disarm(ets_timer: *mut ets_timer) {
    critical_section::with(|_| unsafe {
        if let Some(timer) = TIMERS.iter_mut().find(|t| t.ets_timer == ets_timer) {
            debug!("timer_disarm {:x}", timer.id());
            timer.active = false;
        } else {
            debug!("timer_disarm {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_done(ets_timer: *mut ets_timer) {
    critical_section::with(|_| unsafe {
        if let Some((idx, timer)) = TIMERS
            .iter_mut()
            .enumerate()
            .find(|(_, t)| t.ets_timer == ets_timer)
        {
            debug!("timer_done {:x}", timer.id());
            timer.active = false;

            (*ets_timer).priv_ = core::ptr::null_mut();
            (*ets_timer).expire = 0;

            TIMERS.swap_remove(idx);
        } else {
            debug!("timer_done {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_setfn(
    ets_timer: *mut ets_timer,
    pfunction: unsafe extern "C" fn(*mut c_types::c_void),
    parg: *mut c_types::c_void,
) {
    debug!(
        "timer_setfn {:x} {:?} {:?}",
        ets_timer as usize, pfunction, parg
    );

    let set = critical_section::with(|_| unsafe {
        if let Some(timer) = TIMERS.iter_mut().find(|t| t.ets_timer == ets_timer) {
            timer.callback = TimerCallback::new(pfunction, parg);
            timer.active = false;

            (*ets_timer).expire = 0;

            true
        } else {
            (*ets_timer).next = core::ptr::null_mut();
            (*ets_timer).period = 0;
            (*ets_timer).func = None;
            (*ets_timer).priv_ = core::ptr::null_mut();
            TIMERS
                .push(Timer {
                    ets_timer,
                    started: 0,
                    timeout: 0,
                    active: false,
                    periodic: false,
                    callback: TimerCallback::new(pfunction, parg),
                })
                .is_ok()
        }
    });

    if !set {
        warn!("Failed to set timer function {:x}", ets_timer as usize);
    }
}

pub fn compat_esp_timer_create(
    args: *const esp_timer_create_args_t,
    out_handle: *mut esp_timer_handle_t,
) -> i32 {
    unsafe {
        debug!("esp_timer_create {:?} {:?}", (*args).callback, (*args).arg);
    }

    critical_section::with(|_| unsafe {
        if TIMERS.is_full() {
            // TODO: should we return -1 instead?
            panic!("ran out of timers");
        }

        let ets_timer =
            crate::compat::malloc::calloc(1, core::mem::size_of::<ets_timer>()).cast::<ets_timer>();

        _ = TIMERS.push(Timer {
            ets_timer,
            started: 0,
            timeout: 0,
            active: false,
            periodic: false,
            callback: TimerCallback::from(unwrap!(args.as_ref())),
        });

        debug!("esp_timer_create {:x}", ets_timer as usize);
        *out_handle = ets_timer as _;

        0
    })
}
