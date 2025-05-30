#[cfg(any(esp32c6, esp32h2))]
use peripherals::INTPRI as SystemPeripheral;
#[cfg(not(any(esp32c6, esp32h2)))]
use peripherals::SYSTEM as SystemPeripheral;

use crate::{
    TimeBase,
    hal::{
        interrupt::{self, TrapFrame},
        peripherals::{self, Interrupt},
        riscv,
    },
    preempt_builtin::{task_switch, timer::setup_timebase},
};

pub(crate) fn setup_timer(timer: TimeBase) {
    // make sure the scheduling won't start before everything is setup
    riscv::interrupt::disable();

    setup_timebase(timer);
}

pub(crate) fn setup_multitasking() {
    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR2,
        interrupt::Priority::Priority1,
    ));

    unsafe {
        riscv::interrupt::enable();
    }
}

pub(crate) fn disable_multitasking() {
    interrupt::disable(crate::hal::system::Cpu::ProCpu, Interrupt::FROM_CPU_INTR2);
}

#[unsafe(no_mangle)]
extern "C" fn FROM_CPU_INTR2(trap_frame: &mut TrapFrame) {
    // clear FROM_CPU_INTR3
    SystemPeripheral::regs()
        .cpu_intr_from_cpu_2()
        .modify(|_, w| w.cpu_intr_from_cpu_2().clear_bit());

    task_switch(trap_frame);
}

pub(crate) fn yield_task() {
    SystemPeripheral::regs()
        .cpu_intr_from_cpu_2()
        .modify(|_, w| w.cpu_intr_from_cpu_2().set_bit());
}
