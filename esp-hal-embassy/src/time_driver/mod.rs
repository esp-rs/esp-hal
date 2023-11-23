#[cfg(feature = "time-systimer")]
pub use self::systimer::*;
#[cfg(feature = "time-timg0")]
pub use self::timg0::*;

#[cfg(feature = "time-systimer")]
mod systimer;
#[cfg(feature = "time-timg0")]
mod timg0;

#[cfg(any(feature = "time-systimer", feature = "time-timg0"))]
impl embassy_time::driver::Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        EmbassyTimer::now()
    }

    unsafe fn allocate_alarm(&self) -> Option<embassy_time::driver::AlarmHandle> {
        critical_section::with(|cs| {
            for (i, alarm) in self.alarms.borrow(cs).iter().enumerate() {
                if !alarm.allocated.get() {
                    // set alarm so it is not overwritten
                    alarm.allocated.set(true);
                    self.on_alarm_allocated(i);
                    return Some(embassy_time::driver::AlarmHandle::new(i as u8));
                }
            }

            None
        })
    }

    fn set_alarm_callback(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        callback: fn(*mut ()),
        ctx: *mut (),
    ) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: embassy_time::driver::AlarmHandle, timestamp: u64) -> bool {
        self.set_alarm(alarm, timestamp)
    }
}
