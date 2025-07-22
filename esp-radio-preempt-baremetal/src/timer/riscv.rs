use esp_hal::{
    interrupt::{self, TrapFrame, software::SoftwareInterrupt},
    peripherals::Interrupt,
};

use crate::task::task_switch;

pub(crate) fn setup_multitasking() {
    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR2,
        interrupt::Priority::Priority1,
    ));
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(esp_hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR2);
}

#[unsafe(no_mangle)]
#[esp_hal::ram]
extern "C" fn FROM_CPU_INTR2(trap_frame: &mut TrapFrame) {
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
