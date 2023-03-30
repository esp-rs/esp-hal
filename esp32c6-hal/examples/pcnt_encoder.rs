//! PCNT Encoder Demo
//!
//! This example decodes a quadrature encoder
//!
//! Since the PCNT units reset to zero when they reach their limits
//! we enable an interrupt on the upper and lower limits and
//! track the overflow in an AtomicI32

#![no_std]
#![no_main]
use core::{
    cell::RefCell,
    cmp::min,
    sync::atomic::{AtomicI32, Ordering},
};

use critical_section::Mutex;
use esp32c6_hal as esp_hal;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    interrupt,
    pcnt::{channel, channel::PcntSource, unit, PCNT},
    peripherals::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_println::println;

static UNIT0: Mutex<RefCell<Option<unit::Unit>>> = Mutex::new(RefCell::new(None));
static VALUE: AtomicI32 = AtomicI32::new(0);

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let unit_number = unit::Number::Unit1;

    // setup a pulse couter
    println!("setup pulse counter unit 0");
    let pcnt = PCNT::new(peripherals.PCNT, &mut system.peripheral_clock_control);
    let mut u0 = pcnt.get_unit(unit_number);
    u0.configure(unit::Config {
        low_limit: -100,
        high_limit: 100,
        filter: Some(min(10u16 * 80, 1023u16)),
        ..Default::default()
    })
    .unwrap();

    println!("setup channel 0");
    let mut ch0 = u0.get_channel(channel::Number::Channel0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin_a = io.pins.gpio5.into_pull_up_input();
    let mut pin_b = io.pins.gpio6.into_pull_up_input();

    ch0.configure(
        PcntSource::from_pin(&mut pin_a),
        PcntSource::from_pin(&mut pin_b),
        channel::Config {
            lctrl_mode: channel::CtrlMode::Reverse,
            hctrl_mode: channel::CtrlMode::Keep,
            pos_edge: channel::EdgeMode::Decrement,
            neg_edge: channel::EdgeMode::Increment,
            invert_ctrl: false,
            invert_sig: false,
        },
    );

    println!("setup channel 1");
    let mut ch1 = u0.get_channel(channel::Number::Channel1);
    ch1.configure(
        PcntSource::from_pin(&mut pin_b),
        PcntSource::from_pin(&mut pin_a),
        channel::Config {
            lctrl_mode: channel::CtrlMode::Reverse,
            hctrl_mode: channel::CtrlMode::Keep,
            pos_edge: channel::EdgeMode::Increment,
            neg_edge: channel::EdgeMode::Decrement,
            invert_ctrl: false,
            invert_sig: false,
        },
    );
    println!("subscribing to events");
    u0.events(unit::Events {
        low_limit: true,
        high_limit: true,
        thresh0: false,
        thresh1: false,
        zero: false,
    });

    println!("enabling interrupts");
    u0.listen();
    println!("resume pulse counter unit 0");
    u0.resume();

    critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));

    interrupt::enable(peripherals::Interrupt::PCNT, interrupt::Priority::Priority2).unwrap();

    let mut last_value: i32 = 0;
    loop {
        critical_section::with(|cs| {
            let mut u0 = UNIT0.borrow_ref_mut(cs);
            let u0 = u0.as_mut().unwrap();
            let value: i32 = u0.get_value() as i32 + VALUE.load(Ordering::SeqCst);
            if value != last_value {
                println!("value: {value}");
                last_value = value;
            }
        });
    }
}

#[interrupt]
fn PCNT() {
    critical_section::with(|cs| {
        let mut u0 = UNIT0.borrow_ref_mut(cs);
        let u0 = u0.as_mut().unwrap();
        if u0.interrupt_set() {
            let events = u0.get_events();
            if events.high_limit {
                VALUE.fetch_add(100, Ordering::SeqCst);
            } else if events.low_limit {
                VALUE.fetch_add(-100, Ordering::SeqCst);
            }
            u0.reset_interrupt();
        }
    });
}
