use core::cell::RefCell;

use critical_section::Mutex;
use esp32c2 as pac;
use esp32c2_hal as hal;
use esp32c2_hal::interrupt::TrapFrame;
use esp32c2_hal::prelude::*;
use hal::peripherals::Interrupt;
use hal::systimer::{Alarm, Periodic, Target};

use crate::{binary, preempt::preempt::task_switch};
use log::trace;

pub const TICKS_PER_SECOND: u64 = 16_000_000;

pub const COUNTER_BIT_MASK: u64 = 0x000F_FFFF_FFFF_FFFF;

#[cfg(debug_assertions)]
const TIMER_DELAY: fugit::HertzU32 = fugit::HertzU32::from_raw(500);
#[cfg(not(debug_assertions))]
const TIMER_DELAY: fugit::HertzU32 = fugit::HertzU32::from_raw(1_000);

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, 0>>>> = Mutex::new(RefCell::new(None));

pub fn setup_timer_isr(systimer: Alarm<Target, 0>) {
    let alarm0 = systimer.into_periodic();
    alarm0.set_period(TIMER_DELAY.into());
    alarm0.clear_interrupt();
    alarm0.interrupt_enable(true);

    critical_section::with(|cs| ALARM0.borrow_ref_mut(cs).replace(alarm0));

    esp32c2_hal::interrupt::enable(
        Interrupt::SYSTIMER_TARGET0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    #[cfg(feature = "wifi")]
    esp32c2_hal::interrupt::enable(Interrupt::WIFI_MAC, hal::interrupt::Priority::Priority1)
        .unwrap();

    #[cfg(feature = "wifi")]
    esp32c2_hal::interrupt::enable(Interrupt::WIFI_PWR, hal::interrupt::Priority::Priority1)
        .unwrap();

    #[cfg(feature = "ble")]
    {
        esp32c2_hal::interrupt::enable(Interrupt::LP_TIMER, hal::interrupt::Priority::Priority1)
            .unwrap();
        esp32c2_hal::interrupt::enable(Interrupt::BT_MAC, hal::interrupt::Priority::Priority1)
            .unwrap();
    }

    esp32c2_hal::interrupt::enable(Interrupt::SW_INTR_3, hal::interrupt::Priority::Priority1)
        .unwrap();

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

        trace!("interrupt WIFI_MAC {:p} {:p}", fnc, arg);

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

        trace!("interrupt WIFI_PWR {:p} {:p}", fnc, arg);

        if !fnc.is_null() {
            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
        }

        trace!("interrupt 1 done");
    };
}

#[cfg(feature = "ble")]
#[interrupt]
fn LP_TIMER() {
    unsafe {
        log::trace!("LP_TIMER interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_7;

        trace!("interrupt LP_TIMER {:p} {:p}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt LP_TIMER call");

            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("LP_TIMER done");
        }

        trace!("interrupt LP_TIMER done");
    };
}

#[cfg(feature = "ble")]
#[interrupt]
fn BT_MAC() {
    unsafe {
        log::trace!("BT_MAC interrupt");

        let (fnc, arg) = crate::ble::npl::ble_os_adapter_chip_specific::ISR_INTERRUPT_4;

        trace!("interrupt BT_MAC {:p} {:p}", fnc, arg);

        if !fnc.is_null() {
            trace!("interrupt BT_MAC call");

            let fnc: fn(*mut binary::c_types::c_void) = core::mem::transmute(fnc);
            fnc(arg);
            trace!("BT_MAC done");
        }

        trace!("interrupt BT_MAC done");
    };
}

#[interrupt]
fn SYSTIMER_TARGET0(trap_frame: &mut TrapFrame) {
    // clear the systimer intr
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });

    task_switch(trap_frame);
}

#[interrupt]
fn SW_INTR_3(trap_frame: &mut TrapFrame) {
    unsafe {
        // clear SW_INTR_3
        (&*pac::SYSTEM::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().clear_bit());
    }

    critical_section::with(|cs| {
        let mut alarm0 = ALARM0.borrow_ref_mut(cs);
        let alarm0 = alarm0.as_mut().unwrap();

        alarm0.set_period(TIMER_DELAY.into());
        alarm0.clear_interrupt();
    });

    task_switch(trap_frame);
}

pub fn yield_task() {
    unsafe {
        (&*pac::SYSTEM::PTR)
            .cpu_intr_from_cpu_3
            .modify(|_, w| w.cpu_intr_from_cpu_3().set_bit());
    }
}

/// Current systimer count value
/// A tick is 1 / 16_000_000 seconds
pub fn get_systimer_count() -> u64 {
    critical_section::with(|_| unsafe {
        let systimer = &(*pac::SYSTIMER::ptr());

        systimer.unit0_op.write(|w| w.bits(1 << 30));

        // wait for value available
        loop {
            let valid = (systimer.unit0_op.read().bits() >> 29) & 1;
            if valid != 0 {
                break;
            }
        }

        let value_lo = systimer.unit0_value_lo.read().bits() as u64;
        let value_hi = (systimer.unit0_value_hi.read().bits() as u64) << 32;

        (value_lo | value_hi) as u64
    })
}
