use core::cell::RefCell;

use atomic_polyfill::{AtomicU64, Ordering};
use critical_section::Mutex;
use esp32s2_hal::xtensa_lx;
use esp32s2_hal::xtensa_lx_rt;
use esp32s2_hal::xtensa_lx_rt::exception::Context;
use esp32s2_hal::{
    interrupt,
    peripherals::{self, TIMG1},
    prelude::*,
    timer::{Timer, Timer0},
};
use log::trace;

use crate::preempt::preempt::task_switch;
use esp32s2_hal::macros::interrupt;

pub const TICKS_PER_SECOND: u64 = 40_000_000;

pub const COUNTER_BIT_MASK: u64 = 0xFFFF_FFFF_FFFF_FFFF;

const TIMER_DELAY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

static TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

static TIME: AtomicU64 = AtomicU64::new(0);

pub fn get_systimer_count() -> u64 {
    TIME.load(Ordering::Relaxed) + read_timer_value()
}

#[inline(always)]
fn read_timer_value() -> u64 {
    let value = xtensa_lx::timer::get_cycle_count() as u64;
    value * 40_000_000 / 240_000_000
}

pub fn setup_timer_isr(timg1_timer0: Timer<Timer0<TIMG1>>) {
    let mut timer1 = timg1_timer0;
    interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority2,
    )
    .unwrap();

    #[cfg(feature = "wifi")]
    interrupt::enable(
        peripherals::Interrupt::WIFI_MAC,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    #[cfg(feature = "wifi")]
    interrupt::enable(
        peripherals::Interrupt::WIFI_PWR,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    timer1.listen();
    timer1.start(TIMER_DELAY.into_duration());
    critical_section::with(|cs| {
        TIMER1.borrow_ref_mut(cs).replace(timer1);
    });

    xtensa_lx::timer::set_ccompare0(0xffffffff);

    unsafe {
        let enabled = esp32s2_hal::xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 6 // Timer0
            | 1 << 29 // Software1
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask() | enabled,
        );
    }

    while unsafe { crate::preempt::FIRST_SWITCH.load(core::sync::atomic::Ordering::Relaxed) } {}
}

#[allow(non_snake_case)]
#[no_mangle]
fn Timer0(_level: u32) {
    TIME.fetch_add(0x1_0000_0000 * 40_000_000 / 240_000_000, Ordering::Relaxed);

    xtensa_lx::timer::set_ccompare0(0xffffffff);
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:p} {:p}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:p} {:p}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[interrupt]
fn TG1_T0_LEVEL(context: &mut Context) {
    task_switch(context);

    critical_section::with(|cs| {
        crate::memory_fence::memory_fence();

        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();
        timer.clear_interrupt();
        timer.start(TIMER_DELAY.into_duration());
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn Software1(_level: u32, context: &mut Context) {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.227  {0}", in(reg) intr, options(nostack)); // 227 = "intclear"
    }

    task_switch(context);

    critical_section::with(|cs| {
        crate::memory_fence::memory_fence();

        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();
        timer.clear_interrupt();
        timer.start(TIMER_DELAY.into_duration());
    });
}

pub fn yield_task() {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.226  {0}", in(reg) intr, options(nostack)); // 226 = "intset"
    }
}
