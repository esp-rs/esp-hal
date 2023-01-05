#![feature(stmt_expr_attributes)]
#![no_std]
#![no_main]

// use esp32c6_hal::peripherals;
use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;

use esp32c6_hal::{
    clock::{ClockControl, CpuClock},
    gpio::IO,
    peripherals::{self, Peripherals, UART0},
    prelude::*,
    //timer::TimerGroup,
    //uart::{
    //   config::{Config, DataBits, Parity, StopBits, AtCmdConfig},
    //   TxRxPins,
    //},
    //Uart,
    //Cpu,
};

use nb::block;

use esp_hal_common::system::SystemExt;

use esp_backtrace as _;

// static SERIAL: Mutex<RefCell<Option<Uart<UART0>>>> = Mutex::new(RefCell::new(None));

use esp_println::println;

#[riscv_rt::entry]
fn main() -> ! {
    esp_println::println!("fffff");
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(
        system.clock_control,
        CpuClock::Clock80MHz,
    )
    .freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio5.into_push_pull_output();

    led.set_high().unwrap();

    // let mut serial0 = Uart::new(peripherals.UART0);

    // let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    // let mut timer0 = timer_group0.timer0;
    // let mut wdt0 = timer_group0.wdt;
    // let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    // let mut wdt1 = timer_group1.wdt;

    // let config = Config {
    //     baudrate: 115200,
    //     data_bits: DataBits::DataBits8,
    //     parity: Parity::ParityNone,
    //     stop_bits: StopBits::STOP1,
    // };

    // let pins = TxRxPins::new_tx_rx(
    //     io.pins.gpio1.into_push_pull_output(),
    //     io.pins.gpio2.into_floating_input(),
    // );

    // let mut serial1 = Uart::new_with_config(peripherals.UART1, Some(config), Some(pins), &clocks);

    // timer0.start(250u64.millis());

    // println!("Start");
    // loop {
    //     serial1.write(0x42).ok();
    //     let read = block!(serial1.read());

    //     match read {
    //         Ok(read) => println!("Read {:02x}", read),
    //         Err(err) => println!("Error {:?}", err),
    //     }

    //     block!(timer0.wait()).unwrap();
    // }
    loop{}
}


