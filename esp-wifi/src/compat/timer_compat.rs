use alloc::boxed::Box;

use esp_hal::sync::Locked;

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

#[repr(C)]
#[derive(Debug, Clone)]
pub(crate) struct Timer {
    pub ets_timer: *mut ets_timer,
    pub started: u64,
    pub timeout: u64,
    pub active: bool,
    pub periodic: bool,
    pub callback: TimerCallback,

    next: Option<Box<Timer>>,
}

impl Timer {
    pub(crate) fn id(&self) -> usize {
        self.ets_timer as usize
    }
}

pub(crate) struct TimerQueue {
    head: Option<Box<Timer>>,
}

impl TimerQueue {
    const fn new() -> Self {
        Self { head: None }
    }

    fn find(&mut self, ets_timer: *mut ets_timer) -> Option<&mut Box<Timer>> {
        let mut current = self.head.as_mut();
        while let Some(timer) = current {
            if core::ptr::eq(timer.ets_timer, ets_timer) {
                return Some(timer);
            }
            current = timer.next.as_mut();
        }

        None
    }

    pub(crate) unsafe fn find_next_due(
        &mut self,
        current_timestamp: u64,
    ) -> Option<&mut Box<Timer>> {
        let mut current = self.head.as_mut();
        while let Some(timer) = current {
            if timer.active
                && crate::time::time_diff(timer.started, current_timestamp) >= timer.timeout
            {
                return Some(timer);
            }
            current = timer.next.as_mut();
        }

        None
    }

    fn remove(&mut self, ets_timer: *mut ets_timer) {
        if let Some(head) = self.head.as_mut() {
            if core::ptr::eq(head.ets_timer, ets_timer) {
                self.head = head.next.take();
                return;
            }
        }

        let timer = self.find(ets_timer);
        if let Some(to_remove) = timer {
            let tail = to_remove.next.take();

            let mut current = self.head.as_mut();
            let before = {
                let mut found = None;
                while let Some(before) = current {
                    if core::ptr::eq(before.next.as_mut().unwrap().ets_timer, ets_timer) {
                        found = Some(before);
                        break;
                    }
                    current = before.next.as_mut();
                }
                found
            };

            if let Some(before) = before {
                let to_remove = before.next.take().unwrap();
                let to_remove = Box::into_raw(to_remove);
                unsafe {
                    crate::compat::malloc::free(to_remove as *mut _);
                }
                before.next = tail;
            }
        }
    }

    fn push(&mut self, to_add: Box<Timer>) -> Result<(), ()> {
        if self.head.is_none() {
            self.head = Some(to_add);
            return Ok(());
        }

        let mut current = self.head.as_mut();
        while let Some(timer) = current {
            if timer.next.is_none() {
                timer.next = Some(to_add);
                break;
            }
            current = timer.next.as_mut();
        }
        Ok(())
    }
}

unsafe impl Send for TimerQueue {}

pub(crate) static TIMERS: Locked<TimerQueue> = Locked::new(TimerQueue::new());

pub(crate) fn compat_timer_arm(ets_timer: *mut ets_timer, tmout: u32, repeat: bool) {
    compat_timer_arm_us(ets_timer, tmout * 1000, repeat);
}

pub(crate) fn compat_timer_arm_us(ets_timer: *mut ets_timer, us: u32, repeat: bool) {
    let systick = crate::time::systimer_count();
    let ticks = crate::time::micros_to_ticks(us as u64);

    trace!(
        "timer_arm_us {:x} current: {} ticks: {} repeat: {}",
        ets_timer as usize, systick, ticks, repeat
    );

    TIMERS.with(|timers| {
        if let Some(timer) = timers.find(ets_timer) {
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
        if let Some(timer) = timers.find(ets_timer) {
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
        if let Some(timer) = timers.find(ets_timer) {
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
    pfunction: unsafe extern "C" fn(*mut c_types::c_void),
    parg: *mut c_types::c_void,
) {
    trace!(
        "timer_setfn {:x} {:?} {:?}",
        ets_timer as usize, pfunction, parg
    );
    let set = TIMERS.with(|timers| unsafe {
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

            let timer =
                crate::compat::malloc::calloc(1, core::mem::size_of::<Timer>()) as *mut Timer;
            (*timer).next = None;
            (*timer).ets_timer = ets_timer;
            (*timer).started = 0;
            (*timer).timeout = 0;
            (*timer).active = false;
            (*timer).periodic = false;
            (*timer).callback = TimerCallback::new(pfunction, parg);

            timers.push(Box::from_raw(timer)).is_ok()
        }
    });

    if !set {
        warn!("Failed to set timer function {:x}", ets_timer as usize);
    }
}
