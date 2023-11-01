use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex;

use crate::{
    hal::{
        interrupt::{self, TrapFrame},
        peripherals::{self, Interrupt},
        prelude::*,
        riscv,
        systimer::{Alarm, Periodic, SystemTimer, Target},
    },
    preempt::preempt::task_switch,
};

#[cfg(feature = "esp32c6")]
use peripherals::INTPRI as SystemPeripheral;
#[cfg(not(feature = "esp32c6"))]
use peripherals::SYSTEM as SystemPeripheral;

/// The timer responsible for time slicing.
pub type TimeBase = Alarm<Target, 0>;
static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, 0>>>> = Mutex::new(RefCell::new(None));
const TIMESLICE_FREQUENCY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

// Time keeping

pub const TICKS_PER_SECOND: u64 = 16_000_000;

pub fn setup_timer(systimer: TimeBase) {
    // make sure the scheduling won't start before everything is setup
    unsafe {
        riscv::interrupt::disable();
    }

    let alarm0 = systimer.into_periodic();
    alarm0.set_period(TIMESLICE_FREQUENCY.into());
    alarm0.clear_interrupt();
    alarm0.interrupt_enable(true);

    critical_section::with(|cs| ALARM0.borrow_ref_mut(cs).replace(alarm0));

    unwrap!(interrupt::enable(
        Interrupt::SYSTIMER_TARGET0,
        interrupt::Priority::Priority1,
    ));
}

pub fn setup_multitasking() {
    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR3,
        interrupt::Priority::Priority1,
    ));

    unsafe {
        riscv::interrupt::enable();
    }

    while unsafe { crate::preempt::FIRST_SWITCH.load(Ordering::Relaxed) } {}
}

#[interrupt]
fn SYSTIMER_TARGET0(trap_frame: &mut TrapFrame) {
    // clear the systimer intr
    critical_section::with(|cs| {
        unwrap!(ALARM0.borrow_ref_mut(cs).as_mut()).clear_interrupt();
    });

    task_switch(trap_frame);
}

#[interrupt]
fn FROM_CPU_INTR3(trap_frame: &mut TrapFrame) {
    unsafe {
        // clear FROM_CPU_INTR3
        (&*SystemPeripheral::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().clear_bit());
    }

    critical_section::with(|cs| {
        let mut alarm0 = ALARM0.borrow_ref_mut(cs);
        let alarm0 = unwrap!(alarm0.as_mut());

        alarm0.set_period(TIMESLICE_FREQUENCY.into());
        alarm0.clear_interrupt();
    });

    task_switch(trap_frame);
}

pub fn yield_task() {
    unsafe {
        (&*SystemPeripheral::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().set_bit());
    }
}

/// Current systimer count value
/// A tick is 1 / 16_000_000 seconds
pub fn get_systimer_count() -> u64 {
    SystemTimer::now()
}

// TODO: use an Instance type instead...
pub fn time_diff(start: u64, end: u64) -> u64 {
    // 52-bit wrapping sub
    end.wrapping_sub(start) & 0x000f_ffff_ffff_ffff
}
