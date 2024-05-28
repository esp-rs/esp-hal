use core::cell::RefCell;

use critical_section::Mutex;

use crate::{
    hal::{
        interrupt,
        peripherals::{self, TIMG1},
        prelude::*,
        timer::timg::{Timer, Timer0},
        trapframe::TrapFrame,
        xtensa_lx,
        xtensa_lx_rt,
    },
    preempt::preempt::task_switch,
};

/// The timer responsible for time slicing.
pub type TimeBase = Timer<Timer0<TIMG1>, esp_hal::Blocking>;
static TIMER1: Mutex<RefCell<Option<TimeBase>>> = Mutex::new(RefCell::new(None));
const TIMESLICE_FREQUENCY: fugit::HertzU64 =
    fugit::HertzU64::from_raw(crate::CONFIG.tick_rate_hz as u64);

// Time keeping
pub const TICKS_PER_SECOND: u64 = 1_000_000;

/// This function must not be called in a critical section. Doing so may return
/// an incorrect value.
pub fn get_systimer_count() -> u64 {
    esp_hal::time::current_time().ticks()
}

pub fn setup_timer(timer1: TimeBase) -> Result<(), esp_hal::timer::Error> {
    unsafe {
        interrupt::bind_interrupt(
            peripherals::Interrupt::TG1_T0_LEVEL,
            core::mem::transmute(tg1_t0_level as *const ()),
        );
    }

    unwrap!(interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority2,
    ));

    timer1.listen();
    timer1.load_value(TIMESLICE_FREQUENCY.into_duration())?;
    timer1.start();
    critical_section::with(|cs| {
        TIMER1.borrow_ref_mut(cs).replace(timer1);
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

fn do_task_switch(context: &mut TrapFrame) {
    critical_section::with(|cs| {
        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
        timer
            .load_value(TIMESLICE_FREQUENCY.into_duration())
            .unwrap();
        timer.start();
    });

    task_switch(context);
}

extern "C" fn tg1_t0_level(context: &mut TrapFrame) {
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
