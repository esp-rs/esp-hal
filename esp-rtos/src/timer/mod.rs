use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    time::{Duration, Instant, Rate},
};

#[cfg(feature = "embassy")]
use crate::TIMER_QUEUE;
use crate::{
    SCHEDULER,
    TICK_RATE,
    TimeBase,
    task::{TaskExt, TaskPtr, TaskQueue, TaskState, TaskTimerQueueElement},
};

#[cfg(feature = "embassy")]
pub(crate) mod embassy;

const TIMESLICE_DURATION: Duration = Rate::from_hz(TICK_RATE).as_duration();

pub(crate) struct TimerQueue {
    queue: TaskQueue<TaskTimerQueueElement>,
    next_wakeup: u64,
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
        }
    }

    fn retain(&mut self, now: u64, mut on_task_ready: impl FnMut(TaskPtr)) {
        let mut timer_queue = core::mem::take(self);

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
}

pub(crate) struct TimeDriver {
    timer: TimeBase,
    pub(crate) timer_queue: TimerQueue,
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
        }
    }

    pub(crate) fn handle_alarm(&mut self, on_task_ready: impl FnMut(TaskPtr)) {
        self.timer_queue.retain(crate::now(), on_task_ready);
    }

    pub(crate) fn arm_next_wakeup(&mut self, with_time_slice: bool) {
        let wakeup_at = self.timer_queue.next_wakeup;

        #[cfg(feature = "embassy")]
        let wakeup_at = wakeup_at.min(TIMER_QUEUE.next_wakeup());

        if wakeup_at != u64::MAX {
            let sleep_duration = wakeup_at.saturating_sub(crate::now());

            let mut timeout = if with_time_slice {
                sleep_duration.min(TIMESLICE_DURATION.as_micros())
            } else {
                // assume 52-bit underlying timer. it's not a big deal to sleep for a shorter time
                sleep_duration & ((1 << 52) - 1)
            };

            trace!("Arming timer for {:?}", timeout);
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
        } else if with_time_slice {
            trace!("Arming timer for {:?}", TIMESLICE_DURATION);
            unwrap!(self.timer.schedule(TIMESLICE_DURATION));
        } else {
            trace!("Stopping timer");
            self.timer.stop();
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
    #[cfg(feature = "embassy")]
    TIMER_QUEUE.handle_alarm(crate::now());

    SCHEDULER.with(|scheduler| {
        let time_driver = unwrap!(scheduler.time_driver.as_mut());

        time_driver.timer.clear_interrupt();

        // Process timer queue. This will wake up ready tasks, and set a new alarm.
        time_driver.handle_alarm(|ready_task| {
            debug_assert_eq!(
                ready_task.state(),
                crate::task::TaskState::Sleeping,
                "task: {:?}",
                ready_task
            );

            debug!("Task {:?} is ready", ready_task);

            scheduler.run_queue.mark_task_ready(ready_task);
        });

        // The timer interrupt fires when a task is ready to run, an embassy timer expires or when a
        // time slice tick expires. Trigger a context switch in all cases, which context switch will
        // re-arm the timer.

        // `Scheduler::switch_task` must be called on a single interrupt priority level only.
        // To ensure this, we call yield_task to pend the software interrupt.
        //
        // RISC-V: esp-hal's interrupt handler can process multiple interrupts before handing
        // control back to the interrupted context. This can result in two task switches
        // before the first one's context save could run. To prevent this, here we only
        // trigger the software interrupt which will then run the scheduler.
        //
        // ESP32: Because on ESP32 the software interrupt is triggered at priority 3 but
        // the timer interrupt is triggered at priority 1, we need to trigger the
        // software interrupt manually.
        //
        // The rest of the lineup could switch tasks here, but it's not done to save some IRAM.
        crate::task::yield_task();
    });
}
