//! PCNT Encoder Demo
//!
//! This example decodes a quadrature encoder.
//!
//! Since the PCNT units reset to zero when they reach their limits
//! we enable an interrupt on the upper and lower limits and
//! track the overflow in an AtomicI32
//!
//! The following wiring is assumed:
//! - Control signal => GPIO4
//! - Edge signal    => GPIO5

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::{cell::RefCell, cmp::min, sync::atomic::Ordering};

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Pull},
    interrupt::Priority,
    pcnt::{channel, unit, Pcnt},
    prelude::*,
};
use esp_println::println;
use portable_atomic::AtomicI32;

static UNIT0: Mutex<RefCell<Option<unit::Unit<'static, 1>>>> = Mutex::new(RefCell::new(None));
static VALUE: AtomicI32 = AtomicI32::new(0);

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Set up a pulse counter:
    println!("setup pulse counter unit 0");
    let mut pcnt = Pcnt::new(peripherals.PCNT);
    pcnt.set_interrupt_handler(interrupt_handler);
    let u0 = pcnt.unit1;
    u0.set_low_limit(Some(-100)).unwrap();
    u0.set_high_limit(Some(100)).unwrap();
    u0.set_filter(Some(min(10u16 * 80, 1023u16))).unwrap();
    u0.clear();

    println!("setup channel 0");
    let ch0 = &u0.channel0;

    let pin_a = Input::new(peripherals.pins.gpio4, Pull::Up);
    let pin_b = Input::new(peripherals.pins.gpio5, Pull::Up);

    let (input_a, _) = pin_a.split();
    let (input_b, _) = pin_b.split();

    ch0.set_ctrl_signal(input_a.clone());
    ch0.set_edge_signal(input_b.clone());
    ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
    ch0.set_input_mode(channel::EdgeMode::Increment, channel::EdgeMode::Decrement);

    println!("setup channel 1");
    let ch1 = &u0.channel1;
    ch1.set_ctrl_signal(input_b);
    ch1.set_edge_signal(input_a);
    ch1.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
    ch1.set_input_mode(channel::EdgeMode::Decrement, channel::EdgeMode::Increment);

    println!("enabling interrupts");
    u0.listen();
    println!("resume pulse counter unit 0");
    u0.resume();

    let counter = u0.counter.clone();

    critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));

    let mut last_value: i32 = 0;
    loop {
        let value: i32 = counter.get() as i32 + VALUE.load(Ordering::SeqCst);
        if value != last_value {
            println!("value: {value}");
            last_value = value;
        }
    }
}

#[handler(priority = Priority::Priority2)]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut u0 = UNIT0.borrow_ref_mut(cs);
        let u0 = u0.as_mut().unwrap();
        if u0.interrupt_is_set() {
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
