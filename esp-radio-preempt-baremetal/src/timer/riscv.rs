use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt},
    peripherals::Interrupt,
};

use crate::task::task_switch;

pub(crate) fn setup_multitasking() {
    // unwrap!(interrupt::enable(
    //     Interrupt::FROM_CPU_INTR2,
    //     interrupt::Priority::Priority1,
    // ));

    let swint2_handler = esp_hal::interrupt::InterruptHandler::new_not_nested(
        swint2_handler,
        esp_hal::interrupt::Priority::Priority1,
    );

    unsafe { SoftwareInterrupt::<2>::steal() }.set_interrupt_handler(swint2_handler);
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(esp_hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR2);
}

#[esp_hal::ram]
extern "C" fn swint2_handler() {
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.reset();

    task_switch();
}

#[inline]
pub(crate) fn yield_task() {
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.raise();
}
