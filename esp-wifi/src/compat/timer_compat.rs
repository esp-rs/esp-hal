use core::cell::RefCell;

use critical_section::Mutex;
use esp_wifi_sys::include::malloc;

use crate::binary::{
    c_types,
    include::{esp_timer_create_args_t, ets_timer},
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

    pub(crate) fn call(self) {
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

    next: *mut Timer,
}

impl Timer {
    pub(crate) fn id(&self) -> usize {
        self.ets_timer as usize
    }
}

pub(crate) struct TimerQueue {
    head: *mut Timer,
}

impl TimerQueue {
    const fn new() -> Self {
        Self {
            head: core::ptr::null_mut(),
        }
    }

    fn find(&mut self, ets_timer: *mut ets_timer) -> Option<&mut Timer> {
        let mut current = self.head;

        while !current.is_null() {
            unsafe {
                if (*current).ets_timer == ets_timer {
                    return current.as_mut();
                }
                current = (*current).next;
            }
        }

        None
    }

    pub(crate) unsafe fn find_next_due(&mut self, current_timestamp: u64) -> Option<*mut Timer> {
        let mut current = self.head;

        while !current.is_null() {
            unsafe {
                let timer = *current;
                if timer.active
                    && crate::timer::time_diff(timer.started, current_timestamp) >= timer.timeout
                {
                    return Some(current);
                }
                current = (*current).next;
            }
        }

        None
    }

    fn remove(&mut self, ets_timer: *mut ets_timer) {
        let mut current = self.head;
        let mut prev: *mut Timer = core::ptr::null_mut();

        while !current.is_null() {
            unsafe {
                if (*current).ets_timer == ets_timer {
                    if !prev.is_null() {
                        (*prev).next = (*current).next;
                    } else {
                        self.head = (*current).next;
                    }
                    super::malloc::free(current.cast());
                }
                prev = current;
                current = (*current).next;
            }
        }
    }

    fn push(&mut self, timer: *mut Timer) -> Result<(), ()> {
        let mut current = self.head;

        unsafe {
            while !current.is_null() && !(*current).next.is_null() {
                current = (*current).next;
            }

            if !current.is_null() {
                (*current).next = timer;
            } else {
                self.head = timer;
            }
        }

        Ok(())
    }
}

pub(crate) static mut TIMERS: Mutex<RefCell<TimerQueue>> =
    Mutex::new(RefCell::new(TimerQueue::new()));

pub(crate) fn compat_timer_arm(ets_timer: *mut ets_timer, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout * 1000, repeat);
}

pub(crate) fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    let systick = crate::timer::get_systimer_count();
    let ticks = crate::timer::micros_to_ticks(us as u64);

    trace!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ets_timer as usize,
        systick,
        ticks,
        repeat
    );

    critical_section::with(|cs| unsafe {
        if let Some(timer) = TIMERS.borrow_ref_mut(cs).find(ets_timer) {
            timer.started = systick;
            timer.timeout = ticks;
            timer.active = true;
            timer.periodic = repeat;
        } else {
            trace!("timer_arm_us {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_disarm(ets_timer: *mut ets_timer) {
    critical_section::with(|cs| unsafe {
        if let Some(timer) = TIMERS.borrow_ref_mut(cs).find(ets_timer) {
            trace!("timer_disarm {:x}", timer.id());
            timer.active = false;
        } else {
            trace!("timer_disarm {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_done(ets_timer: *mut ets_timer) {
    critical_section::with(|cs| unsafe {
        let mut timers = TIMERS.borrow_ref_mut(cs);
        if let Some(timer) = timers.find(ets_timer) {
            trace!("timer_done {:x}", timer.id());
            timer.active = false;

            (*ets_timer).priv_ = core::ptr::null_mut();
            (*ets_timer).expire = 0;

            timers.remove(ets_timer);
        } else {
            trace!("timer_done {:x} not found", ets_timer as usize);
        }
    })
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

    let set = critical_section::with(|cs| unsafe {
        let mut timers = TIMERS.borrow_ref_mut(cs);
        if let Some(timer) = timers.find(ets_timer) {
            timer.callback = TimerCallback::new(pfunction, parg);
            timer.active = false;

            (*ets_timer).expire = 0;

            true
        } else {
            (*ets_timer).next = core::ptr::null_mut();
            (*ets_timer).period = 0;
            (*ets_timer).func = None;
            (*ets_timer).priv_ = core::ptr::null_mut();

            let timer = malloc((core::mem::size_of::<Timer>()) as u32) as *mut Timer;
            (*timer).next = core::ptr::null_mut();
            (*timer).ets_timer = ets_timer;
            (*timer).started = 0;
            (*timer).timeout = 0;
            (*timer).active = false;
            (*timer).periodic = false;
            (*timer).callback = TimerCallback::new(pfunction, parg);

            timers.push(timer).is_ok()
        }
    });

    if !set {
        warn!("Failed to set timer function {:x}", ets_timer as usize);
    }
}
