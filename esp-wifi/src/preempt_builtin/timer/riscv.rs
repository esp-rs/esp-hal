use esp_hal::interrupt::InterruptHandler;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
use peripherals::INTPRI as SystemPeripheral;
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
use peripherals::SYSTEM as SystemPeripheral;

use crate::{
    hal::{
        interrupt::{self, TrapFrame},
        peripherals::{self, Interrupt},
        riscv,
        time::Rate,
    },
    preempt_builtin::task_switch,
    TimeBase,
};

/// The timer responsible for time slicing.
const TIMESLICE_FREQUENCY: Rate = Rate::from_hz(crate::CONFIG.tick_rate_hz);

use super::TIMER;

pub(crate) fn setup_timer(mut alarm0: TimeBase) {
    // make sure the scheduling won't start before everything is setup
    riscv::interrupt::disable();

    let cb: extern "C" fn() = unsafe { core::mem::transmute(handler as *const ()) };
    alarm0.set_interrupt_handler(InterruptHandler::new(cb, interrupt::Priority::Priority1));
    unwrap!(alarm0.start(TIMESLICE_FREQUENCY.as_duration()));
    TIMER.with(|timer| {
        alarm0.enable_interrupt(true);
        timer.replace(alarm0);
    });
}

pub(crate) fn disable_timer() {
    TIMER.with(|timer| {
        let timer = unwrap!(timer.as_mut());
        timer.enable_interrupt(false);
        unwrap!(timer.cancel());
    });
}

pub(crate) fn setup_multitasking() {
    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR3,
        interrupt::Priority::Priority1,
    ));

    unsafe {
        riscv::interrupt::enable();
    }
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(crate::hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR3);
}

extern "C" fn handler(trap_frame: &mut TrapFrame) {
    // clear the systimer intr
    TIMER.with(|timer| {
        unwrap!(timer.as_mut()).clear_interrupt();
    });

    task_switch(trap_frame);
}

#[no_mangle]
extern "C" fn FROM_CPU_INTR3(trap_frame: &mut TrapFrame) {
    // clear FROM_CPU_INTR3
    SystemPeripheral::regs()
        .cpu_intr_from_cpu_3()
        .modify(|_, w| w.cpu_intr_from_cpu_3().clear_bit());

    TIMER.with(|alarm0| {
        let alarm0 = unwrap!(alarm0.as_mut());
        alarm0.clear_interrupt();
    });

    task_switch(trap_frame);
}

pub(crate) fn yield_task() {
    SystemPeripheral::regs()
        .cpu_intr_from_cpu_3()
        .modify(|_, w| w.cpu_intr_from_cpu_3().set_bit());
}
