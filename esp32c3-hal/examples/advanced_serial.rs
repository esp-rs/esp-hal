//! This shows how to configure UART
//! You can short the TX and RX pin and see it reads what was written.
//! Additionally you can connect a logic analzyer to TX and see how the changes
//! of the configuration change the output signal.

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    serial::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    timer::TimerGroup,
    RtcCntl,
    Serial,
    IO,
};
use esp_println::println;
use nb::block;
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_global_enable(false);
    wdt0.disable();
    wdt1.disable();

    let config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_floating_input(),
    );

    let mut serial1 = Serial::new_with_config(peripherals.UART1, Some(config), Some(pins), &clocks);

    timer0.start(250u64.millis());

    println!("Start");
    loop {
        serial1.write(0x42).ok();
        let read = block!(serial1.read());

        match read {
            Ok(read) => println!("Read {:02x}", read),
            Err(err) => println!("Error {:?}", err),
        }

        block!(timer0.wait()).unwrap();
    }
}
