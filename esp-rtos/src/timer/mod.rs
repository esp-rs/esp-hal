use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    system::Cpu,
    time::{Duration, Instant, Rate},
};

#[cfg(feature = "embassy")]
use crate::TIMER_QUEUE;
#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
use crate::{
    SCHEDULER,
    TICK_RATE,
    TimeBase,
    scheduler::Scheduler,
    task::{TaskExt, TaskPtr, TaskQueue, TaskState, TaskTimerQueueElement},
};

#[cfg(feature = "embassy")]
pub(crate) mod embassy;

const TIMESLICE_DURATION: Duration = Rate::from_hz(TICK_RATE).as_duration();

pub(crate) struct TimerQueue {
    queue: TaskQueue<TaskTimerQueueElement>,
    next_wakeup: u64,
    time_slice_target: [u64; Cpu::COUNT],
}

impl Default for TimerQueue {
    fn default() -> Self {
        // Can't derive Default, the default implementation must start with no wakeup timestamp
        Self::new()
    }
}

impl TimerQueue {
    pub const fn new() -> Self {
        Self {
            queue: TaskQueue::new(),
            next_wakeup: u64::MAX,
            time_slice_target: [u64::MAX; Cpu::COUNT],
        }
    }

    fn retain(&mut self, now: u64, mut on_task_ready: impl FnMut(TaskPtr)) {
        if now < self.next_wakeup {
            trace!("Skipping timer queue");
            return;
        }

        let mut timer_queue = core::mem::take(self);
        self.time_slice_target = timer_queue.time_slice_target;

        while let Some(mut task_ptr) = timer_queue.pop() {
            let task = unsafe { task_ptr.as_mut() };

            let wakeup_at = task.wakeup_at;
            let ready = wakeup_at <= now;

            if ready {
                on_task_ready(task_ptr);
            } else {
                self.push(task_ptr, wakeup_at);
            }
        }
    }

    pub fn push(&mut self, task: TaskPtr, wakeup_at: u64) {
        self.queue.push(task);

        self.next_wakeup = self.next_wakeup.min(wakeup_at);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        // We can allow waking up sooner than necessary, so this function doesn't need to
        // re-calculate the next wakeup time.
        self.queue.pop()
    }

    pub fn remove(&mut self, task: TaskPtr) {
        // We can allow waking up sooner than necessary, so this function doesn't need to
        // re-calculate the next wakeup time.
        self.queue.remove(task);
    }

    fn next_wakeup(&self) -> u64 {
        let mut wakeup_at = self.next_wakeup;

        for time_slice_target in self.time_slice_target.iter().copied() {
            wakeup_at = wakeup_at.min(time_slice_target);
        }

        #[cfg(feature = "embassy")]
        let wakeup_at = wakeup_at.min(TIMER_QUEUE.next_wakeup());

        wakeup_at
    }
}

pub(crate) struct TimeDriver {
    timer: TimeBase,
    pub(crate) timer_queue: TimerQueue,
    current_alarm: u64,
}

impl TimeDriver {
    pub(crate) fn new(mut timer: TimeBase) -> Self {
        // The timer needs to tick at Priority 1 to prevent accidentally interrupting
        // priority limited locks.
        let timer_priority = Priority::Priority1;

        let cb: extern "C" fn() = unsafe { core::mem::transmute(timer_tick_handler as *const ()) };

        cfg_if::cfg_if! {
            if #[cfg(riscv)] {
                // Register the interrupt handler without nesting to satisfy the requirements of the
                // task switching code
                let handler = InterruptHandler::new_not_nested(cb, timer_priority);
            } else {
                let handler = InterruptHandler::new(cb, timer_priority);
            }
        };

        timer.set_interrupt_handler(handler);
        timer.listen();

        Self {
            timer,
            timer_queue: TimerQueue::new(),
            current_alarm: u64::MAX,
        }
    }

    pub(crate) fn handle_alarm(&mut self, now: u64, on_task_ready: impl FnMut(TaskPtr)) {
        if now < self.current_alarm {
            trace!(
                "Not processing RTOS timer queue. Now: {}, expected next wakeup: {}",
                now, self.current_alarm
            );
            return;
        }
        trace!("Processing RTOS timer queue at {}", now);
        self.current_alarm = u64::MAX;
        self.timer_queue.retain(now, on_task_ready);
    }

    pub(crate) fn set_time_slice(&mut self, cpu: Cpu, now: u64, enable: bool) {
        self.timer_queue.time_slice_target[cpu as usize] = if enable {
            trace!("Enable time slicing");
            now + TIMESLICE_DURATION.as_micros()
        } else {
            trace!("Disable time slicing");
            u64::MAX
        };
    }

    pub(crate) fn arm_next_wakeup(&mut self, now: u64) {
        let next_wakeup = self.timer_queue.next_wakeup();

        // Only skip arming timer if the timestamp is the same. If the next wakeup changed to a
        // later timestamp, the tick handler may not trigger a scheduler run. This means that if we
        // did not arm here, the timer would not be re-armed.
        if next_wakeup == self.current_alarm {
            return;
        }

        self.current_alarm = next_wakeup;

        let sleep_duration = next_wakeup.saturating_sub(now);

        // assume 52-bit underlying timer. it's not a big deal to sleep for a shorter time
        let mut timeout = sleep_duration & ((1 << 52) - 1);

        trace!("Arming timer for {} (target = {})", timeout, next_wakeup);
        loop {
            match self.timer.schedule(Duration::from_micros(timeout)) {
                Ok(_) => break,
                Err(esp_hal::timer::Error::InvalidTimeout) if timeout != 0 => {
                    timeout /= 2;
                    continue;
                }
                Err(e) => panic!("Failed to schedule timer: {:?}", e),
            }
        }
    }

    pub(crate) fn schedule_wakeup(&mut self, mut current_task: TaskPtr, at: Instant) -> bool {
        debug_assert_eq!(
            current_task.state(),
            TaskState::Ready,
            "task: {:?}",
            current_task
        );

        // Target time is infinite, suspend task without waking up via timer.
        if at == Instant::EPOCH + Duration::MAX {
            current_task.set_state(TaskState::Sleeping);
            debug!("Suspending task: {:?}", current_task);
            return true;
        }

        // Target time is in the past, don't sleep.
        if at <= Instant::now() {
            debug!("Target time is in the past");
            return false;
        }

        current_task.set_state(TaskState::Sleeping);

        let timestamp = at.duration_since_epoch().as_micros();
        debug!(
            "Scheduling wakeup for task {:?} at timestamp {}",
            current_task, timestamp
        );
        self.timer_queue.push(current_task, timestamp);

        unsafe { current_task.as_mut().wakeup_at = timestamp };

        true
    }
}

#[esp_hal::ram]
extern "C" fn timer_tick_handler() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::marker_begin(TraceEvents::TimerTickHandler as u32);

    trace!("Timer tick");

    SCHEDULER.with_shared(|scheduler| {
        let now = crate::now();

        #[cfg(feature = "embassy")]
        {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::marker_begin(TraceEvents::ProcessEmbassyTimerQueue as u32);

            TIMER_QUEUE.handle_alarm(now);

            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::marker_end(TraceEvents::ProcessEmbassyTimerQueue as u32);
        }

        let mut scheduler = unwrap!(scheduler.try_borrow_mut());
        let scheduler = &mut *scheduler;

        let time_driver = unwrap!(scheduler.time_driver.as_mut());

        time_driver.timer.clear_interrupt();

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker_begin(TraceEvents::ProcessTimerQueue as u32);

        // Process timer queue. This will wake up ready tasks, and set a new alarm.
        time_driver.handle_alarm(now, |ready_task| {
            debug_assert_eq!(
                ready_task.state(),
                crate::task::TaskState::Sleeping,
                "task: {:?}",
                ready_task
            );

            debug!("Task {:?} is ready", ready_task);

            if scheduler.run_queue.mark_task_ready(ready_task) {
                Scheduler::trigger_schedule(
                    &mut scheduler.per_cpu,
                    &mut scheduler.run_queue,
                    ready_task,
                );
            }
        });

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker_end(TraceEvents::ProcessTimerQueue as u32);

        if now >= time_driver.timer_queue.time_slice_target[0] {
            crate::task::yield_task();
        }

        #[cfg(multi_core)]
        if now >= time_driver.timer_queue.time_slice_target[1] {
            crate::task::schedule_other_core();
        }
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::marker_end(TraceEvents::TimerTickHandler as u32);
}
