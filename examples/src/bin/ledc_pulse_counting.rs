//! Generates a PWM signal on LEDC channel 0 with a duty cycle of 50% and a
//! frequency of 24 kHz. The PWM signal will be generated for 100 cycles and
//! then the signal will be disabled.
//!
//! The following wiring is assumed:
//! - LED => GPIO0

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    gpio::{GpioPin, Io},
    interrupt::Priority,
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, LSClockSource, Timer, TimerBuilder},
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
    },
    prelude::*,
};
use esp_println as _;
use static_cell::StaticCell;

const DESIRED_PULSE_COUNT: u16 = 100;

static CHANNEL0: Mutex<RefCell<Option<Channel<'_, LowSpeed, GpioPin<4>>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = io.pins.gpio4;

    let mut ledc = Ledc::new(peripherals.LEDC);

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    ledc.set_interrupt_handler(interrupt_handler);

    let lstimer0 = {
        static LSTIMER0: StaticCell<Timer<LowSpeed>> = StaticCell::new();
        LSTIMER0.init_with(|| {
            TimerBuilder::<LowSpeed>::new(
                &ledc,
                timer::Number::Timer0,
                timer::config::Duty::Duty5Bit,
                LSClockSource::LedcPwmClk,
                24.kHz(),
            )
            .unwrap()
            .build()
        })
    };

    let channel0 = ledc.get_channel(channel::Number::Channel0, led);

    channel0.enable_counter_with_overflow(DESIRED_PULSE_COUNT);
    critical_section::with(|cs| {
        CHANNEL0.borrow_ref_mut(cs).replace(channel0);
        let mut channel0 = CHANNEL0.borrow(cs).borrow_mut();
        let channel0 = channel0.as_mut().unwrap();
        channel0.enable_overflow_interrupt();
        channel0
            .configure(channel::config::Config {
                timer: lstimer0,
                duty_pct: 50,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();
    });

    loop {}
}

#[handler(priority = Priority::Priority1)]
#[ram]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let channel0 = CHANNEL0.borrow(cs).borrow();
        if let Some(channel0) = channel0.as_ref() {
            if channel0.is_overflow_interrupt_active() {
                // Disable the signal output and clear the overflow interrupt
                // Note that the timer is not stopped
                channel0.disable_signal_output();
                channel0.clear_overflow_interrupt();
            }
        }
    })
}
