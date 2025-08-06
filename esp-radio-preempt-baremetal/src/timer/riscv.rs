use core::ptr::addr_of_mut;

use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt},
    peripherals::Interrupt,
};

use crate::task::task_switch;

static mut FAKE_TRAP_FRAME: esp_hal::interrupt::TrapFrame = esp_hal::interrupt::TrapFrame {
    ra: 0xdeadbeef,
    t0: 0xdeadbeef,
    t1: 0xdeadbeef,
    t2: 0xdeadbeef,
    t3: 0xdeadbeef,
    t4: 0xdeadbeef,
    t5: 0xdeadbeef,
    t6: 0xdeadbeef,
    a0: 0xdeadbeef,
    a1: 0xdeadbeef,
    a2: 0xdeadbeef,
    a3: 0xdeadbeef,
    a4: 0xdeadbeef,
    a5: 0xdeadbeef,
    a6: 0xdeadbeef,
    a7: 0xdeadbeef,
};

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
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.reset();

    task_switch(unsafe { &mut *addr_of_mut!(FAKE_TRAP_FRAME) });
}

#[inline]
pub(crate) fn yield_task() {
    // clear FROM_CPU_INTR2
    let swi = unsafe { SoftwareInterrupt::<2>::steal() };
    swi.raise();
}

#[esp_hal::ram]
pub(crate) extern "C" fn timer_tick_handler() {
    super::clear_timer_interrupt();
    crate::task::task_switch(unsafe { &mut *addr_of_mut!(FAKE_TRAP_FRAME) });
}
