use core::{cell::Cell, ptr};

use embassy_time::driver::{AlarmHandle, Driver};

#[cfg_attr(
    all(systimer, feature = "embassy-time-systick",),
    path = "time_driver_systimer.rs"
)]
#[cfg_attr(
    all(timg0, feature = "embassy-time-timg0"),
    path = "time_driver_timg.rs"
)]
mod time_driver;

use time_driver::EmbassyTimer;

use crate::clock::Clocks;

pub fn init(clocks: &Clocks, td: time_driver::TimerType) {
    EmbassyTimer::init(clocks, td)
}

pub struct AlarmState {
    pub timestamp: Cell<u64>,

    // This is really a Option<(fn(*mut ()), *mut ())>
    // but fn pointers aren't allowed in const yet
    pub callback: Cell<*const ()>,
    pub ctx: Cell<*mut ()>,
    pub allocated: Cell<bool>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    pub const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            callback: Cell::new(ptr::null()),
            ctx: Cell::new(ptr::null_mut()),
            allocated: Cell::new(false),
        }
    }
}

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        EmbassyTimer::now()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        return critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            for i in 0..time_driver::ALARM_COUNT {
                let c = alarms.get_unchecked(i);
                if !c.allocated.get() {
                    // set alarm so it is not overwritten
                    c.allocated.set(true);
                    return Option::Some(AlarmHandle::new(i as u8));
                }
            }
            return Option::None;
        });
    }

    fn set_alarm_callback(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        callback: fn(*mut ()),
        ctx: *mut (),
    ) {
        critical_section::with(|cs| {
            let alarm = unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) };
            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }

    fn set_alarm(&self, alarm: embassy_time::driver::AlarmHandle, timestamp: u64) -> bool {
        self.set_alarm(alarm, timestamp)
    }
}
