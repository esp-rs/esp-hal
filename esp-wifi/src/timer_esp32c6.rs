use core::cell::RefCell;

use critical_section::Mutex;
use esp32c6 as pac;
use esp32c6_hal as hal;
use esp32c6_hal::prelude::*;
use esp32c6_hal::trapframe::TrapFrame;
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

    esp32c6_hal::interrupt::enable(
        Interrupt::SYSTIMER_TARGET0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    #[cfg(feature = "wifi")]
    esp32c6_hal::interrupt::enable(Interrupt::WIFI_MAC, hal::interrupt::Priority::Priority1)
        .unwrap();

    #[cfg(feature = "wifi")]
    esp32c6_hal::interrupt::enable(Interrupt::WIFI_PWR, hal::interrupt::Priority::Priority1)
        .unwrap();

    // make sure to disable WIFI_BB by mapping it to CPU interrupt 31 which is masked by default
    // for some reason for this interrupt, mapping it to 0 doesn't deactivate it
    let interrupt_core0 = unsafe { &*esp32c6::INTERRUPT_CORE0::PTR };
    interrupt_core0
        .wifi_bb_intr_map
        .write(|w| w.wifi_bb_intr_map().variant(31));

    #[cfg(feature = "ble")]
    {
        esp32c6_hal::interrupt::enable(Interrupt::RWBT, hal::interrupt::Priority::Priority1)
            .unwrap();
        esp32c6_hal::interrupt::enable(Interrupt::RWBLE, hal::interrupt::Priority::Priority1)
            .unwrap();
        esp32c6_hal::interrupt::enable(Interrupt::BT_BB, hal::interrupt::Priority::Priority1)
            .unwrap();
    }

    esp32c6_hal::interrupt::enable(
        Interrupt::CPU_FROM_CPU_3,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    unsafe {
        esp32c6_hal::riscv::interrupt::enable();
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
fn RWBT() {
    unsafe {
        let intr = &*pac::INTERRUPT_CORE0::ptr();
        intr.cpu_int_clear.write(|w| w.bits(1 << 1));

        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt RWBT {:p} {:p}", fnc, arg);

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
        let intr = &*pac::INTERRUPT_CORE0::ptr();
        intr.cpu_int_clear.write(|w| w.bits(1 << 1));

        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION5;

        trace!("interrupt RWBLE {:p} {:p}", fnc, arg);

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
        let intr = &*pac::INTERRUPT_CORE0::ptr();
        intr.cpu_int_clear.write(|w| w.bits(1 << 1));

        let (fnc, arg) = crate::ble::btdm::ble_os_adapter_chip_specific::BT_INTERRUPT_FUNCTION8;

        trace!("interrupt BT_BB {:p} {:p}", fnc, arg);

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
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });

    task_switch(trap_frame);
}

#[interrupt]
fn CPU_FROM_CPU_3(trap_frame: &mut TrapFrame) {
    unsafe {
        // clear FROM_CPU_INTR3
        (&*pac::INTPRI::PTR)
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
        (&*pac::INTPRI::PTR)
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
