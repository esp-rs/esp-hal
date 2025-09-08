use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    time::{Duration, Instant, Rate},
};

use crate::{
    SCHEDULER,
    TICK_RATE,
    TimeBase,
    task::{Context, TaskPtr, TaskQueue, TaskTimerQueueElement},
};

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

        Self {
            timer,
            timer_queue: TimerQueue::new(),
        }
    }

    pub(crate) fn start(&mut self) {
        self.timer.listen();
    }

    pub(crate) fn stop(&mut self) {
        self.timer.unlisten();
        self.timer.stop();
    }

    pub(crate) fn handle_alarm(&mut self, mut on_task_ready: impl FnMut(&mut Context)) {
        self.timer.clear_interrupt();

        let mut timer_queue = core::mem::take(&mut self.timer_queue);

        let now = Instant::now().duration_since_epoch().as_micros();

        while let Some(mut task_ptr) = timer_queue.pop() {
            let task = unsafe { task_ptr.as_mut() };

            let wakeup_at = task.wakeup_at;
            let ready = wakeup_at >= now;

            if ready {
                on_task_ready(task);
            } else {
                self.timer_queue.push(task_ptr, wakeup_at);
            }
        }
    }

    pub(crate) fn arm_next_wakeup(&mut self, with_time_slice: bool) {
        let now = Instant::now().duration_since_epoch().as_micros();
        let wakeup_at = self.timer_queue.next_wakeup;

        if wakeup_at != u64::MAX {
            let sleep_duration = wakeup_at.saturating_sub(now);

            let timeout = if with_time_slice {
                sleep_duration.min(TIMESLICE_DURATION.as_micros())
            } else {
                sleep_duration
            };

            unwrap!(self.timer.schedule(Duration::from_micros(timeout)));
        } else if with_time_slice {
            unwrap!(self.timer.schedule(TIMESLICE_DURATION));
        } else {
            self.timer.stop();
        }
    }
}

#[esp_hal::ram]
extern "C" fn timer_tick_handler(#[cfg(xtensa)] _context: &mut esp_hal::trapframe::TrapFrame) {
    SCHEDULER.with(|_scheduler| {
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
        cfg_if::cfg_if! {
            if #[cfg(any(riscv, esp32))] {
                SCHEDULER.yield_task();
            } else {
                _scheduler.switch_task(_context)
            }
        }
    });
}
