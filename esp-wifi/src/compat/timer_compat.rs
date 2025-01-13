use esp_hal::sync::Locked;

use crate::{
    binary::{
        c_types::c_void,
        include::{esp_timer_create_args_t, ets_timer},
    },
    compat::malloc::MallocBox as Box,
};

#[derive(Clone, Copy, Debug)]
pub(crate) struct TimerCallback {
    f: unsafe extern "C" fn(*mut c_void),
    args: *mut c_void,
}

impl TimerCallback {
    fn new(f: unsafe extern "C" fn(*mut c_void), args: *mut c_void) -> Self {
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

#[repr(C)]
#[derive(Clone)]
pub(crate) struct Timer {
    pub ets_timer: *mut ets_timer,
    pub started: u64,
    pub timeout: u64,
    pub active: bool,
    pub periodic: bool,
    pub callback: TimerCallback,
}

struct TimerItem {
    timer: Timer,
    next: Option<Box<TimerItem>>,
}

impl Timer {
    pub(crate) fn new(
        ets_timer: *mut ets_timer,
        pfunction: unsafe extern "C" fn(*mut c_void),
        parg: *mut c_void,
    ) -> Self {
        unsafe {
            (*ets_timer).next = core::ptr::null_mut();
            (*ets_timer).period = 0;
            (*ets_timer).func = None;
            (*ets_timer).priv_ = core::ptr::null_mut();
        }

        Self {
            ets_timer,
            started: 0,
            timeout: 0,
            active: false,
            periodic: false,
            callback: TimerCallback::new(pfunction, parg),
        }
    }

    fn expired(&self, current_timestamp: u64) -> bool {
        self.active && crate::timer::time_diff(self.started, current_timestamp) >= self.timeout
    }
}

impl Timer {
    pub(crate) fn id(&self) -> usize {
        self.ets_timer as usize
    }
}

pub(crate) struct TimerQueue {
    head: Option<Box<TimerItem>>,
}

impl TimerQueue {
    const fn new() -> Self {
        Self { head: None }
    }

    fn find_item(&mut self, p: impl Fn(&TimerItem) -> bool) -> Option<&mut TimerItem> {
        let mut current = &mut self.head;
        while let Some(timer) = current {
            if p(timer) {
                return Some(timer);
            }
            current = &mut timer.next;
        }

        None
    }

    fn find_ets_timer(&mut self, ets_timer: *mut ets_timer) -> Option<&mut Timer> {
        self.find_item(|timer| timer.timer.ets_timer == ets_timer)
            .map(|item| &mut item.timer)
    }

    pub(crate) fn find_next_due(&mut self, current_timestamp: u64) -> Option<&mut Timer> {
        self.find_item(|timer| timer.timer.expired(current_timestamp))
            .map(|item| &mut item.timer)
    }

    fn remove(&mut self, ets_timer: *mut ets_timer) {
        if let Some(prev) = self.find_item(|timer| {
            if let Some(next) = timer.next.as_ref() {
                next.timer.ets_timer == ets_timer
            } else {
                false
            }
        }) {
            // We are removing a timer in the middle/end of the list
            let mut to_remove = unwrap!(prev.next.take());
            prev.next = to_remove.next.take();
            trace!("removed timer {:x}", to_remove.timer.ets_timer as usize);
        } else if let Some(mut head) = self.head.take() {
            // We are removing the head or the timer is not in the list
            let next_head = if head.timer.ets_timer == ets_timer {
                trace!("removed timer {:x}", ets_timer as usize);
                head.next.take()
            } else {
                // Replace head, it's not what we wanted to remove
                Some(head)
            };
            self.head = next_head;
        }
    }

    fn push(&mut self, timer: Timer) {
        self.head = Some(Box::new(TimerItem {
            timer,
            next: self.head.take(),
        }));
    }
}

unsafe impl Send for TimerQueue {}

pub(crate) static TIMERS: Locked<TimerQueue> = Locked::new(TimerQueue::new());

pub(crate) fn compat_timer_arm(ets_timer: *mut ets_timer, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout * 1000, repeat);
}

pub(crate) fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    let systick = crate::timer::systimer_count();
    let ticks = crate::timer::micros_to_ticks(us as u64);

    trace!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ets_timer as usize,
        systick,
        ticks,
        repeat
    );

    TIMERS.with(|timers| {
        if let Some(timer) = timers.find_ets_timer(ets_timer) {
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
    trace!("timer disarm");
    TIMERS.with(|timers| {
        if let Some(timer) = timers.find_ets_timer(ets_timer) {
            trace!("timer_disarm {:x}", timer.id());
            timer.active = false;
        } else {
            trace!("timer_disarm {:x} not found", ets_timer as usize);
        }
    })
}

pub fn compat_timer_done(ets_timer: *mut ets_timer) {
    trace!("timer done");
    TIMERS.with(|timers| {
        if let Some(timer) = timers.find_ets_timer(ets_timer) {
            trace!("timer_done {:x}", timer.id());
            timer.active = false;

            unsafe {
                (*ets_timer).priv_ = core::ptr::null_mut();
                (*ets_timer).expire = 0;
            }

            timers.remove(ets_timer);
        } else {
            trace!("timer_done {:x} not found", ets_timer as usize);
        }
    })
}

pub(crate) fn compat_timer_setfn(
    ets_timer: *mut ets_timer,
    pfunction: unsafe extern "C" fn(*mut c_void),
    parg: *mut c_void,
) {
    trace!(
        "timer_setfn {:x} {:?} {:?}",
        ets_timer as usize,
        pfunction,
        parg
    );
    TIMERS.with(|timers| unsafe {
        if let Some(timer) = timers.find_ets_timer(ets_timer) {
            timer.callback = TimerCallback::new(pfunction, parg);
            timer.active = false;

            (*ets_timer).expire = 0;
        } else {
            timers.push(Timer::new(ets_timer, pfunction, parg));
        }
    });
}
