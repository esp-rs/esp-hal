use core::cell::RefCell;

use critical_section::Mutex;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
use peripherals::INTPRI as SystemPeripheral;
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
use peripherals::SYSTEM as SystemPeripheral;

use crate::{
    hal::{
        interrupt::{self, TrapFrame},
        peripherals::{self, Interrupt},
        prelude::*,
        riscv,
        timer::systimer::{Alarm, Periodic, Target},
    },
    preempt::preempt::task_switch,
};

/// The timer responsible for time slicing.
pub type TimeBase = Alarm<Target, esp_hal::Blocking, 0>;
static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, esp_hal::Blocking, 0>>>> =
    Mutex::new(RefCell::new(None));
const TIMESLICE_FREQUENCY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

// Time keeping
pub const TICKS_PER_SECOND: u64 = 1_000_000;

pub fn setup_timer(systimer: TimeBase) -> Result<(), esp_hal::timer::Error> {
    // make sure the scheduling won't start before everything is setup
    riscv::interrupt::disable();

    let alarm0 = systimer.into_periodic();
    alarm0.set_period(TIMESLICE_FREQUENCY.into_duration());
    alarm0.clear_interrupt();
    alarm0.enable_interrupt(true);

    critical_section::with(|cs| ALARM0.borrow_ref_mut(cs).replace(alarm0));

    unsafe {
        interrupt::bind_interrupt(
            Interrupt::SYSTIMER_TARGET0,
            core::mem::transmute::<*const (), unsafe extern "C" fn()>(
                systimer_target0 as *const (),
            ),
        );
    }

    unwrap!(interrupt::enable(
        Interrupt::SYSTIMER_TARGET0,
        interrupt::Priority::Priority1,
    ));
    Ok(())
}

pub fn setup_multitasking() {
    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR3,
        interrupt::Priority::Priority1,
    ));

    unsafe {
        riscv::interrupt::enable();
    }
}

extern "C" fn systimer_target0(trap_frame: &mut TrapFrame) {
    // clear the systimer intr
    critical_section::with(|cs| {
        unwrap!(ALARM0.borrow_ref_mut(cs).as_mut()).clear_interrupt();
    });

    task_switch(trap_frame);
}

#[no_mangle]
extern "C" fn FROM_CPU_INTR3(trap_frame: &mut TrapFrame) {
    unsafe {
        // clear FROM_CPU_INTR3
        (*SystemPeripheral::PTR)
            .cpu_intr_from_cpu_3()
            .modify(|_, w| w.cpu_intr_from_cpu_3().clear_bit());
    }

    critical_section::with(|cs| {
        let alarm0 = ALARM0.borrow_ref(cs);
        let alarm0 = unwrap!(alarm0.as_ref());

        alarm0.set_period(TIMESLICE_FREQUENCY.into_duration());
        alarm0.clear_interrupt();
    });

    task_switch(trap_frame);
}

pub fn yield_task() {
    unsafe {
        (*SystemPeripheral::PTR)
            .cpu_intr_from_cpu_3()
            .modify(|_, w| w.cpu_intr_from_cpu_3().set_bit());
    }
}

/// Current systimer count value
/// A tick is 1 / 1_000_000 seconds
pub fn get_systimer_count() -> u64 {
    esp_hal::time::current_time().ticks()
}

// TODO: use an Instance type instead...
pub fn time_diff(start: u64, end: u64) -> u64 {
    // 52-bit wrapping sub
    end.wrapping_sub(start) & 0x000f_ffff_ffff_ffff
}
