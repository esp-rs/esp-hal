//! Play songs through a piezo-electric buzzer plugged on GPIO6.
//!
//! This assumes that a piezo-electric buzzer is connected to the pin assigned
//! to `buzzer`. (GPIO6)

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal-buzzer

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    ledc::{channel, timer, LSGlobalClkSource, Ledc},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_hal_buzzer::{songs, Buzzer};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut ledc = Ledc::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut buzzer = Buzzer::new(
        &ledc,
        timer::Number::Timer0,
        channel::Number::Channel1,
        io.pins.gpio6,
        &clocks,
    );

    buzzer.play_song(songs::DOOM).unwrap();
    buzzer.play_song(songs::FURELISE).unwrap();
    buzzer.play_song(songs::MERRY_CHRISTMAS).unwrap();
    buzzer.play_song(songs::MII_CHANNEL).unwrap();
    buzzer.play_song(songs::NEVER_GONNA_GIVE_YOU_UP).unwrap();
    buzzer.play_song(songs::ODE_TO_JOY).unwrap();
    buzzer.play_song(songs::PACMAN).unwrap();
    buzzer.play_song(songs::STAR_WARS).unwrap();
    buzzer.play_song(songs::SUPER_MARIO_BROS).unwrap();
    buzzer.play_song(songs::TAKE_ON_ME).unwrap();
    buzzer.play_song(songs::TETRIS).unwrap();
    buzzer.play_song(songs::THE_LION_SLEEPS_TONIGHT).unwrap();
    buzzer.play_song(songs::ZELDA_LULLABY).unwrap();
    buzzer.play_song(songs::ZELDA_THEME).unwrap();

    println!("Done");

    loop {}
}
