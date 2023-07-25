//! Demonstrates generating pulse sequences with RMT
//! Connect a logic analyzer to GPIO1 to see the generated pulses.

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_hal_common::{
    rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
    Rmt,
};

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

    let rmt = Rmt::new(peripherals.RMT, 8u32.MHz(), &mut clock_control, &clocks).unwrap();

    let mut channel = rmt
        .channel0
        .configure(
            io.pins.gpio1,
            TxChannelConfig {
                clk_divider: 255,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    let mut delay = Delay::new(&clocks);

    let mut data = [PulseCode {
        level1: true,
        length1: 200,
        level2: false,
        length2: 50,
    }; 20];

    data[data.len() - 2] = PulseCode {
        level1: true,
        length1: 3000,
        level2: false,
        length2: 500,
    };
    data[data.len() - 1] = PulseCode::default();

    loop {
        let transaction = channel.transmit(&data);
        channel = transaction.wait().unwrap();
        delay.delay_ms(500u32);
    }
}
