use atomic_polyfill::AtomicU32;
use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex;

use crate::{
    hal::{
        interrupt,
        macros::interrupt,
        peripherals::{self, TIMG1},
        prelude::*,
        timer::{Timer, Timer0},
        trapframe::TrapFrame,
        xtensa_lx, xtensa_lx_rt,
    },
    preempt::preempt::task_switch,
};

/// The timer responsible for time slicing.
pub type TimeBase = Timer<Timer0<TIMG1>>;
static TIMER1: Mutex<RefCell<Option<TimeBase>>> = Mutex::new(RefCell::new(None));
const TIMESLICE_FREQUENCY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

// Time keeping

static TIMER_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
pub const CPU_CLOCK: u64 = 240_000_000; // ESP32, ESP32-S2 and ESP32-S3 all have 240MHz CPU clocks.
pub const CLOCK_CYCLES_PER_TICK: u64 = 8;
pub const TICKS_PER_SECOND: u64 = CPU_CLOCK / CLOCK_CYCLES_PER_TICK;

/// This function must not be called in a critical section. Doing so may return an incorrect value.
pub fn get_systimer_count() -> u64 {
    // We read the cycle counter twice to detect overflows.
    // If we don't detect an overflow, we use the TIMER_OVERFLOWS count we read between.
    // If we detect an overflow, we read the TIMER_OVERFLOWS count again to make sure we use the
    // value after the overflow has been handled.

    let counter_before = xtensa_lx::timer::get_cycle_count();
    let mut overflow = TIMER_OVERFLOWS.load(Ordering::Relaxed);
    let counter_after = xtensa_lx::timer::get_cycle_count();

    if counter_after < counter_before {
        overflow = TIMER_OVERFLOWS.load(Ordering::Relaxed);
    }

    (((overflow as u64) << 32) + counter_after as u64) / CLOCK_CYCLES_PER_TICK
}

pub fn setup_timer(mut timer1: TimeBase) {
    unwrap!(interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority2,
    ));

    timer1.listen();
    timer1.start(TIMESLICE_FREQUENCY.into_duration());
    critical_section::with(|cs| {
        TIMER1.borrow_ref_mut(cs).replace(timer1);
    });

    // Set up the time keeping timer.
    xtensa_lx::timer::set_ccompare0(0xffffffff);
}

pub fn setup_multitasking() {
    unsafe {
        let enabled = xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 6 // Timer0
            | 1 << 29 // Software1
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask() | enabled,
        );
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn Timer0(_level: u32) {
    TIMER_OVERFLOWS.fetch_add(1, Ordering::Relaxed);

    xtensa_lx::timer::set_ccompare0(0xffffffff);
}

fn do_task_switch(context: &mut TrapFrame) {
    critical_section::with(|cs| {
        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
        timer.start(TIMESLICE_FREQUENCY.into_duration());
    });

    task_switch(context);
}

#[interrupt]
fn TG1_T0_LEVEL(context: &mut TrapFrame) {
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
