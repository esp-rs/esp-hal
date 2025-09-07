use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    time::{Duration, Rate},
};

use crate::{SCHEDULER, TICK_RATE, TimeBase};

const TIMESLICE_DURATION: Duration = Rate::from_hz(TICK_RATE).as_duration();

pub(crate) struct TimeDriver {
    timer: TimeBase,
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

        Self { timer }
    }

    pub(crate) fn start(&mut self) {
        self.timer.listen();
    }

    pub(crate) fn stop(&mut self) {
        self.timer.unlisten();
        self.timer.stop();
    }

    pub(crate) fn handle_alarm(&mut self) {
        self.timer.clear_interrupt();
        // TODO: we should run through the timer queue here, handle expired timers and calculate
        // next timer wakeup.
    }

    pub(crate) fn arm_next_wakeup(&mut self, with_time_slice: bool) {
        if with_time_slice {
            let timeout = TIMESLICE_DURATION;
            unwrap!(self.timer.schedule(timeout));
        } else {
            self.timer.stop();
        }
    }
}

#[esp_hal::ram]
extern "C" fn timer_tick_handler(#[cfg(xtensa)] _context: &mut esp_hal::trapframe::TrapFrame) {
    SCHEDULER.with(|scheduler| {
        unwrap!(scheduler.time_driver.as_mut()).handle_alarm();

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
                scheduler.switch_task(_context)
            }
        }
    });
}
