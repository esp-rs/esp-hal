use core::cell::RefCell;

use critical_section::Mutex;

use crate::hal::interrupt::{self, TrapFrame};
use crate::hal::peripherals::{self, Interrupt};
use crate::hal::prelude::*;
use crate::hal::riscv;
use crate::hal::systimer::{Alarm, Periodic, SystemTimer, Target};

use crate::{binary, preempt::preempt::task_switch};

pub const TICKS_PER_SECOND: u64 = 16_000_000;

pub const COUNTER_BIT_MASK: u64 = 0x000F_FFFF_FFFF_FFFF;

const TIMER_DELAY: fugit::HertzU32 = fugit::HertzU32::from_raw(crate::CONFIG.tick_rate_hz);

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, 0>>>> = Mutex::new(RefCell::new(None));

pub fn setup_timer_isr(systimer: Alarm<Target, 0>) {
    let alarm0 = systimer.into_periodic();
    alarm0.set_period(TIMER_DELAY.into());
    alarm0.clear_interrupt();
    alarm0.interrupt_enable(true);

    critical_section::with(|cs| ALARM0.borrow_ref_mut(cs).replace(alarm0));

    unwrap!(interrupt::enable(
        Interrupt::SYSTIMER_TARGET0,
        interrupt::Priority::Priority1,
    ));

    #[cfg(feature = "wifi")]
    unwrap!(interrupt::enable(
        Interrupt::WIFI_MAC,
        interrupt::Priority::Priority1
    ));

    #[cfg(feature = "wifi")]
    unwrap!(interrupt::enable(
        Interrupt::WIFI_PWR,
        interrupt::Priority::Priority1
    ));

    #[cfg(feature = "ble")]
    {
        unwrap!(interrupt::enable(
            Interrupt::RWBT,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::RWBLE,
            interrupt::Priority::Priority1
        ));
        unwrap!(interrupt::enable(
            Interrupt::BT_BB,
            interrupt::Priority::Priority1
        ));
    }

    unwrap!(interrupt::enable(
        Interrupt::FROM_CPU_INTR3,
        interrupt::Priority::Priority1,
    ));

    unsafe {
        riscv::interrupt::enable();
    }

    while unsafe { crate::preempt::FIRST_SWITCH.load(core::sync::atomic::Ordering::Relaxed) } {}
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_MAC() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_MAC {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[cfg(feature = "wifi")]
#[interrupt]
fn WIFI_PWR() {
    unsafe {
        let (fnc, arg) = crate::wifi::os_adapter::ISR_INTERRUPT_1;

        trace!("interrupt WIFI_PWR {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[cfg(feature = "ble")]
#[interrupt]
fn RWBT() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt RWBT {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 5 done");
    };
}

#[cfg(feature = "ble")]
#[interrupt]
fn RWBLE() {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt RWBLE {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 5 done");
    };
}

#[cfg(feature = "ble")]
#[interrupt]
fn BT_BB(_trap_frame: &mut TrapFrame) {
    unsafe {
        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION8;

        trace!("interrupt BT_BB {:?} {:?}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 8 done");
    };
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
        (&*peripherals::SYSTEM::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().clear_bit());
    }

    critical_section::with(|cs| {
        let mut alarm0 = ALARM0.borrow_ref_mut(cs);
        let alarm0 = unwrap!(alarm0.as_mut());

        alarm0.set_period(TIMER_DELAY.into());
        alarm0.clear_interrupt();
    });

    task_switch(trap_frame);
}

pub fn yield_task() {
    unsafe {
        (&*peripherals::SYSTEM::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().set_bit());
    }
}

/// Current systimer count value
/// A tick is 1 / 16_000_000 seconds
pub fn get_systimer_count() -> u64 {
    SystemTimer::now()
}
