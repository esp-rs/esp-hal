//! Demonstrates decoding pulse sequences with RMT
//! This uses the boot button as input - press the button a couple of
//! times to generate a pulse sequence and then wait for the idle timeout to see
//! the recorded pulse sequence

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    rmt::{PulseCode, RxChannel, RxChannelConfig, RxChannelCreator},
    timer::TimerGroup,
    Delay,
    Rmt,
    Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};

const WIDTH: usize = 80;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut clock_control = system.peripheral_clock_control;

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, &mut clock_control);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, &mut clock_control);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let rmt = Rmt::new(peripherals.RMT, 1u32.MHz(), &mut clock_control, &clocks).unwrap();

    let mut channel = rmt
        .channel2
        .configure(
            io.pins.gpio9,
            RxChannelConfig {
                clk_divider: 255,
                idle_threshold: 10000,
                ..RxChannelConfig::default()
            },
        )
        .unwrap();

    let mut delay = Delay::new(&clocks);

    let mut data = [PulseCode {
        level1: true,
        length1: 1,
        level2: false,
        length2: 1,
    }; 48];

    loop {
        let transaction = channel.receive(&mut data).unwrap();
        channel = transaction.wait().unwrap();

        let mut total = 0usize;
        for entry in &data[..data.len()] {
            if entry.length1 == 0 {
                break;
            }
            total += entry.length1 as usize;

            if entry.length2 == 0 {
                break;
            }
            total += entry.length2 as usize;
        }

        for entry in &data[..data.len()] {
            if entry.length1 == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length1 as usize);
            let c = if entry.level1 { '-' } else { '_' };
            for _ in 0..count + 1 {
                print!("{}", c);
            }

            if entry.length2 == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length2 as usize);
            let c = if entry.level2 { '-' } else { '_' };
            for _ in 0..count + 1 {
                print!("{}", c);
            }
        }

        println!();
        println!();
        println!();

        delay.delay_ms(500u32);
    }
}
