use core::cell::RefCell;

use atomic_polyfill::{AtomicU32, Ordering};
use critical_section::Mutex;
use esp32_hal::trapframe::TrapFrame;
use esp32_hal::{
    interrupt,
    peripherals::{self, TIMG1},
    prelude::*,
    timer::{Timer, Timer0},
    xtensa_lx, xtensa_lx_rt,
};

use crate::preempt::preempt::task_switch;
use esp32_hal::macros::interrupt;

pub const TICKS_PER_SECOND: u64 = 40_000_000;

pub const COUNTER_BIT_MASK: u64 = 0xFFFF_FFFF_FFFF_FFFF;

const TIMER_DELAY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

static TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

static TIMER_OVERFLOWS: AtomicU32 = AtomicU32::new(0);

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

    (((overflow as u64) << 32) + counter_after as u64) * 40_000_000 / 240_000_000
}

pub fn setup_timer_isr(timg1_timer0: Timer<Timer0<TIMG1>>) {
    let mut timer1 = timg1_timer0;
    unwrap!(interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority2,
    ));

    #[cfg(feature = "wifi")]
    unwrap!(interrupt::enable(
        peripherals::Interrupt::WIFI_MAC,
        interrupt::Priority::Priority1,
    ));

    #[cfg(feature = "ble")]
    {
        unwrap!(interrupt::enable(
            peripherals::Interrupt::RWBT,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::BT_BB,
            interrupt::Priority::Priority1,
        ));

        // It's a mystery why these interrupts are enabled now since it worked without this before
        // Now at least without disabling these nothing will work
        interrupt::disable(esp32_hal::Cpu::ProCpu, peripherals::Interrupt::ETH_MAC);
        interrupt::disable(esp32_hal::Cpu::ProCpu, peripherals::Interrupt::UART0);
    }

    timer1.listen();
    timer1.start(TIMER_DELAY.into_duration());
    critical_section::with(|cs| {
        TIMER1.borrow_ref_mut(cs).replace(timer1);
    });

    xtensa_lx::timer::set_ccompare0(0xffffffff);

    unsafe {
        let enabled = xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 6 // Timer0
            | 1 << 29 // Software1
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask() | enabled,
        );
    }

    while unsafe { crate::preempt::FIRST_SWITCH.load(core::sync::atomic::Ordering::Relaxed) } {}
}

#[cfg(feature = "ble")]
#[allow(non_snake_case)]
#[no_mangle]
fn Software0(_level: u32) {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_7;
        trace!("interrupt Software0 {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn Timer0(_level: u32) {
    TIMER_OVERFLOWS.fetch_add(1, Ordering::Relaxed);

    xtensa_lx::timer::set_ccompare0(0xffffffff);
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;
        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[interrupt]
fn RWBT() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBT {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[interrupt]
fn RWBLE() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_5;
        trace!("interrupt RWBLE {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[cfg(feature = "ble")]
#[interrupt]
fn BT_BB() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::ISR_INTERRUPT_8;
        trace!("interrupt BT_BB {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut crate::binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }
    }
}

#[interrupt]
fn TG1_T0_LEVEL(context: &mut TrapFrame) {
    task_switch(context);

    critical_section::with(|cs| {
        crate::memory_fence::memory_fence();

        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
        timer.start(TIMER_DELAY.into_duration());
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn Software1(_level: u32, context: &mut TrapFrame) {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intclear  {0}", in(reg) intr, options(nostack));
    }

    task_switch(context);

    critical_section::with(|cs| {
        crate::memory_fence::memory_fence();

        let mut timer = TIMER1.borrow_ref_mut(cs);
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
        timer.start(TIMER_DELAY.into_duration());
    });
}

pub fn yield_task() {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intset  {0}", in(reg) intr, options(nostack));
    }
}
