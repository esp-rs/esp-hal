//! Demonstrates decoding pulse sequences with RMT
//! Connect GPIO15 to GPIO4

#![no_std]
#![no_main]

use esp32s2_hal::{
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
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut clock_control = system.peripheral_clock_control;

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, &mut clock_control);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut out = io.pins.gpio15.into_push_pull_output();

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &mut clock_control, &clocks).unwrap();

    let mut channel = rmt
        .channel0
        .configure(
            io.pins.gpio4,
            RxChannelConfig {
                clk_divider: 1,
                idle_threshold: 0b111_1111_1111_1111,
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
        for x in data.iter_mut() {
            x.length1 = 0;
            x.length2 = 0;
        }

        let transaction = channel.receive(&mut data).unwrap();

        // simulate input
        for i in 0u32..5u32 {
            out.set_high().unwrap();
            delay.delay_us(i * 10 + 20);
            out.set_low().unwrap();
            delay.delay_us(i * 20 + 20);
        }

        match transaction.wait() {
            Ok(channel_res) => {
                channel = channel_res;
                for entry in &data[..data.len()] {
                    if entry.length1 == 0 {
                        break;
                    }
                    println!("{} {}", entry.level1, entry.length1);

                    if entry.length2 == 0 {
                        break;
                    }
                    println!("{} {}", entry.level2, entry.length2);
                }
                println!();
            }
            Err((_err, channel_res)) => {
                channel = channel_res;
                println!("Error");
            }
        }

        delay.delay_ms(1500u32);
    }
}
