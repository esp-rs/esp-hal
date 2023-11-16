//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles GPIO1. The
//! current value is printed by the HP core.
//!
//! Make sure to first compile the `esp32c6-lp-hal/examples/blinky.rs` example

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::lp_gpio::IntoLowPowerPin,
    i2c::{lp_i2c::LpI2c, I2C},
    lp_core,
    peripherals::Peripherals,
    prelude::*,
    IO,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        lp_uart::LpUart,
        TxRxPins,
    },
    Uart,
};
use esp_backtrace as _;
use esp_hal_common::system::PeripheralClockControl;
use esp_println::{print, println};
use fugit::HertzU32;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    println!("PIN {:?}", unsafe {
        (0x60091074 as *mut u32).read_volatile()
    });

    let lp_sda = io.pins.gpio6.into_low_power().into_puinput_ppoutput(); // into_puinput_ppoutput
    let lp_scl = io.pins.gpio7.into_low_power().into_puinput_ppoutput(); // into_puinput_ppoutput

    let lp_i2c = LpI2c::new(peripherals.LP_I2C0, lp_sda, lp_scl, 100u32.kHz());

    println!("LP_CLK_CONF {:?}", unsafe {
        (0x600b0400 as *mut u32).read_volatile()
    });
    println!("LPPERI {:?}", unsafe {
        (0x600b0428 as *mut u32).read_volatile()
    });
    println!("I2C_CLK_CONF {:?}", unsafe {
        (0x600b1854 as *mut u32).read_volatile()
    });

    println!("GPIO_OUT {:?}", unsafe {
        (0x60091004 as *mut u32).read_volatile()
    });
    println!("OUT_W1TS {:?}", unsafe {
        (0x60091008 as *mut u32).read_volatile()
    });
    println!("PIN {:?}", unsafe {
        (0x60091074 as *mut u32).read_volatile()
    });
    println!("ENABLE {:?}", unsafe {
        (0x60091020 as *mut u32).read_volatile()
    });

    unsafe {
        (0x60091074 as *mut u32).write_volatile((0x60091074 as *mut u32).read_volatile() | 2097152);

        println!("PIN {:?}", unsafe {
            (0x60091074 as *mut u32).read_volatile()
        })
    };

    // panic!("FF");

    let mut lp_core = esp32c6_hal::lp_core::LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code =
        load_lp_code!("../esp32c6-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/i2c");

        let lp_tx = io.pins.gpio5.into_low_power().into_push_pull_output();
        let lp_rx = io.pins.gpio4.into_low_power().into_floating_input();
    
        let lp_uart = LpUart::new(peripherals.LP_UART, lp_tx, lp_rx);
    
    // start LP core
    lp_core_code.run(&mut lp_core, lp_core::LpCoreWakeupSource::HpCpu, lp_i2c);
    println!("lpcore run");

    let data = (0x5000_2000) as *mut u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
