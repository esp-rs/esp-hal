use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt},
    peripherals::Interrupt,
};

use crate::task::task_switch;

pub(crate) fn setup_multitasking() {
    // Register the interrupt handler without nesting to satisfy the requirements of the task
    // switching code
    let swint2_handler = esp_hal::interrupt::InterruptHandler::new_not_nested(
        unsafe { core::mem::transmute::<*const (), extern "C" fn()>(swint2_handler as *const ()) },
        esp_hal::interrupt::Priority::Priority1,
    );

    unsafe { SoftwareInterrupt::<2>::steal() }.set_interrupt_handler(swint2_handler);
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(esp_hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR2);
}

#[esp_hal::ram]
extern "C" fn swint2_handler() {
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.reset();

    task_switch();
}

#[inline]
pub(crate) fn yield_task() {
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.raise();

    // It takes a bit for the software interrupt to be serviced.
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
}

#[esp_hal::ram]
pub(crate) extern "C" fn timer_tick_handler() {
    super::clear_timer_interrupt();
    // `task_switch` must be called from a single place only. esp-hal's interrupt handler can
    // process multiple interrupts before handing control back to the interrupted context. This can
    // result in two task switches before the first one's context save could run. To prevent this,
    // here we only trigger the software interrupt which will then run the scheduler.
    yield_task();
}
