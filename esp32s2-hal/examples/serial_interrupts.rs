//! This shows some of the interrupts that can be generated by UART/Serial.
//! Use a proper serial terminal to connect to the board (espmonitor and
//! espflash won't work)

#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;
use esp32s2_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals, UART0},
    prelude::*,
    uart::config::AtCmdConfig,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use xtensa_atomic_emulation_trap as _;
use nb::block;
use xtensa_lx_rt::entry;

static SERIAL: Mutex<RefCell<Option<Uart<UART0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the TIMG watchdog timer.
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    let mut serial0 = Uart::new(peripherals.UART0);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt0.disable();
    wdt1.disable();
    rtc.rwdt.disable();

    serial0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
    serial0.set_rx_fifo_full_threshold(30);
    serial0.listen_at_cmd();
    serial0.listen_rx_fifo_full();

    interrupt::enable(peripherals::Interrupt::UART0, interrupt::Priority::Priority2).unwrap();

    timer0.start(1u64.secs());

    critical_section::with(|cs| SERIAL.borrow_ref_mut(cs).replace(serial0));

    loop {
        critical_section::with(|cs| {
            let mut serial = SERIAL.borrow_ref_mut(cs);
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Hello World! Send a single `#` character or send at least 30 characters and see the interrupts trigger.").ok();
        });

        block!(timer0.wait()).unwrap();
    }
}

#[interrupt]
fn UART0() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial = serial.as_mut().unwrap();

        let mut cnt = 0;
        while let nb::Result::Ok(_c) = serial.read() {
            cnt += 1;
        }
        writeln!(serial, "Read {} bytes", cnt,).ok();

        writeln!(
            serial,
            "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
            serial.at_cmd_interrupt_set(),
            serial.rx_fifo_full_interrupt_set(),
        )
        .ok();

        serial.reset_at_cmd_interrupt();
        serial.reset_rx_fifo_full_interrupt();
    });
}

#[xtensa_lx_rt::exception]
fn exception(cause: xtensa_lx_rt::exception::ExceptionCause, frame: xtensa_lx_rt::exception::Context) {
    use esp_println::*;

    println!("\n\nException occured {:?} {:x?}", cause, frame);
    
    let backtrace = esp_backtrace::arch::backtrace();
    for b in backtrace.iter() {
        if let Some(addr) = b {
            println!("0x{:x}", addr)
        }
    }
}