use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt},
    peripherals::Interrupt,
};

use crate::task::task_switch;

pub(crate) fn setup_multitasking() {
    // Register the interrupt handler without nesting to satisfy the requirements of the task
    // switching code
    let swint2_handler = esp_hal::interrupt::InterruptHandler::new_not_nested(
        unsafe { core::mem::transmute(swint2_handler as *const ()) },
        esp_hal::interrupt::Priority::Priority1,
    );

    unsafe { SoftwareInterrupt::<2>::steal() }.set_interrupt_handler(swint2_handler);
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(esp_hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR2);
}

#[esp_hal::ram]
extern "C" fn swint2_handler(trap_frame: &mut esp_hal::interrupt::TrapFrame) {
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.reset();

    task_switch(trap_frame);
}

#[inline]
pub(crate) fn yield_task() {
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.raise();
}

#[esp_hal::ram]
pub(crate) extern "C" fn timer_tick_handler(trap_frame: &mut esp_hal::interrupt::TrapFrame) {
    super::clear_timer_interrupt();
    crate::task::task_switch(trap_frame);
}
