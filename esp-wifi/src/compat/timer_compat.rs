use esp_hal::sync::Locked;
use esp_wifi_sys::c_types::c_void;
use timer_queue::TimerQueue;

use crate::{
    binary::{c_types, include::ets_timer},
    timer::systimer_count,
};

// Usage of ets_timer
// next => callback params
// expire => 1 = armed, 0 = disarmed
// period => repeat period in us or 0
// func => callback function
// priv_ => Timer handle in TimerQueue
#[repr(transparent)]
struct EtsTimer(*mut ets_timer);

impl EtsTimer {
    fn timer_handle(&self) -> timer_queue::Timer {
        unsafe { core::mem::transmute((*(self.0)).priv_) }
    }

    fn set_timer_handle(&mut self, timer: timer_queue::Timer) {
        unsafe {
            (*(self.0)).priv_ = core::mem::transmute(timer);
        }
    }

    fn callback(&self) -> (Option<unsafe extern "C" fn(*mut c_void)>, *mut c_void) {
        unsafe {
            let f = (*(self.0)).func;
            let p = (*(self.0)).next.cast();
            (f, p)
        }
    }

    fn set_callback(
        &mut self,
        func: Option<unsafe extern "C" fn(*mut c_void)>,
        param: *mut c_void,
    ) {
        unsafe {
            (*(self.0)).func = func;
            (*(self.0)).next = param.cast();
        }
    }

    fn armed(&self) -> bool {
        unsafe { (*(self.0)).expire != 0 }
    }

    fn set_armed(&mut self, armed: bool) {
        unsafe {
            (*(self.0)).expire = armed.into();
        }
    }

    fn period(&self) -> u32 {
        unsafe { (*(self.0)).period }
    }

    fn set_period(&mut self, us: u32) {
        unsafe {
            (*(self.0)).period = us;
        }
    }

    fn is_repeating(&self) -> bool {
        self.period() != 0
    }
}

struct ScheduledTimer {
    timeout: u64,
    ets_timer: EtsTimer,
}

static TIMERS: Locked<TimerQueue<ScheduledTimer>> = Locked::new(TimerQueue::new());

pub(crate) fn compat_timer_arm(ets_timer: *mut ets_timer, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout * 1000, repeat);
}

pub(crate) fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    compat_timer_arm_us_with_drift(ets_timer, us, repeat, 0);
}

fn compat_timer_arm_us_with_drift(ets_timer: *mut ets_timer, us: u32, repeat: bool, drift: u64) {
    compat_timer_disarm(ets_timer);

    let systick = crate::timer::systimer_count();
    let ticks = crate::timer::micros_to_ticks(us as u64);

    trace!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ets_timer as usize,
        systick,
        ticks,
        repeat
    );

    let timeout = systick + ticks - drift;

    TIMERS.with(|timers| {
        let timer = timers.insert(
            timeout,
            ScheduledTimer {
                timeout,
                ets_timer: EtsTimer(ets_timer),
            },
        );

        let mut ets_timer = EtsTimer(ets_timer);
        ets_timer.set_timer_handle(timer);
        ets_timer.set_period(if repeat { us } else { 0 });
        ets_timer.set_armed(true);
    })
}

pub fn compat_timer_disarm(ets_timer: *mut ets_timer) {
    trace!("timer disarm {:x}", ets_timer as usize);
    TIMERS.with(|timers| {
        let mut timer = EtsTimer(ets_timer);
        if timer.armed() {
            timers.remove(timer.timer_handle());
        }
        timer.set_armed(false);
    })
}

pub fn compat_timer_done(ets_timer: *mut ets_timer) {
    trace!("timer done");

    let mut timer = EtsTimer(ets_timer);
    timer.set_callback(None, core::ptr::null_mut());
    compat_timer_disarm(ets_timer);
}

pub(crate) fn compat_timer_setfn(
    ets_timer: *mut ets_timer,
    pfunction: unsafe extern "C" fn(*mut c_types::c_void),
    parg: *mut c_types::c_void,
) {
    trace!(
        "timer_setfn {:x} {:?} {:?}",
        ets_timer as usize,
        pfunction,
        parg
    );

    let mut timer = EtsTimer(ets_timer);
    unsafe {
        timer.set_callback(core::mem::transmute(pfunction), core::mem::transmute(parg));
    }
    timer.set_armed(false);
}

pub(crate) fn get_next_due_timer(
) -> Option<(Option<unsafe extern "C" fn(*mut c_void)>, *mut c_void)> {
    let current_timestamp = systimer_count();
    let next_due = TIMERS.with(|timers| {
        let next_due = timers.poll(current_timestamp);

        if let Some(mut next_due) = next_due {
            let (f, p) = next_due.ets_timer.callback();

            if next_due.ets_timer.armed() {
                next_due.ets_timer.set_armed(false);
                Some((next_due, f, p))
            } else {
                None
            }
        } else {
            None
        }
    });

    if let Some((scheduled_timer, func, param)) = next_due {
        let ets_timer = scheduled_timer.ets_timer;
        if ets_timer.is_repeating() {
            // reschedule repeating alarm - take the drift into account
            let drift = systimer_count() - scheduled_timer.timeout;
            compat_timer_arm_us_with_drift(ets_timer.0, ets_timer.period(), true, drift);
        }

        Some((func, param))
    } else {
        None
    }
}
