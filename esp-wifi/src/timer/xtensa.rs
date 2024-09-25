use esp_hal::interrupt::InterruptHandler;

use crate::{
    hal::{interrupt, trapframe::TrapFrame, xtensa_lx, xtensa_lx_rt},
    preempt::task_switch,
    TimeBase,
};

/// The timer responsible for time slicing.
const TIMESLICE_FREQUENCY: fugit::HertzU64 =
    fugit::HertzU64::from_raw(crate::CONFIG.tick_rate_hz as u64);

use super::TIMER;

// Time keeping
pub const TICKS_PER_SECOND: u64 = 1_000_000;

/// This function must not be called in a critical section. Doing so may return
/// an incorrect value.
pub fn get_systimer_count() -> u64 {
    esp_hal::time::now().ticks()
}

pub fn setup_timer(mut timer1: TimeBase) -> Result<(), esp_hal::timer::Error> {
    timer1.set_interrupt_handler(InterruptHandler::new(
        unsafe { core::mem::transmute::<*const (), extern "C" fn()>(handler as *const ()) },
        interrupt::Priority::Priority2,
    ));
    timer1.start(TIMESLICE_FREQUENCY.into_duration())?;
    critical_section::with(|cs| {
        timer1.enable_interrupt(true);
        TIMER.borrow_ref_mut(cs).replace(timer1);
    });
    Ok(())
}

pub fn disable_timer() -> Result<(), esp_hal::timer::Error> {
    critical_section::with(|cs| {
        unwrap!(TIMER.borrow_ref_mut(cs).as_mut()).enable_interrupt(false);
        unwrap!(TIMER.borrow_ref_mut(cs).as_mut()).cancel().unwrap();
    });

    Ok(())
}

pub fn setup_multitasking() {
    unsafe {
        let enabled = xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 29 // Software1
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask() | enabled,
        );
    }
}

pub fn disable_multitasking() {
    let config = unsafe { xtensa_lx::interrupt::enable() };
    xtensa_lx::interrupt::disable_mask(!config);
    xtensa_lx::interrupt::disable_mask(
        1 << 29 // Disable Software1
            | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
            | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask(),
    );
}

fn do_task_switch(context: &mut TrapFrame) {
    critical_section::with(|cs| {
        let mut timer = TIMER.borrow_ref_mut(cs);
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
    });

    task_switch(context);
}

extern "C" fn handler(context: &mut TrapFrame) {
    do_task_switch(context);
}

#[allow(non_snake_case)]
#[no_mangle]
fn Software1(_level: u32, context: &mut TrapFrame) {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intclear  {0}", in(reg) intr, options(nostack));
    }

    do_task_switch(context);
}

pub fn yield_task() {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intset  {0}", in(reg) intr, options(nostack));
    }
}

// TODO: use an Instance type instead...
pub fn time_diff(start: u64, end: u64) -> u64 {
    end.wrapping_sub(start)
}
