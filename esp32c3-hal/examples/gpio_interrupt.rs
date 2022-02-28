#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use bare_metal::Mutex;
use esp32c3_hal::{
    gpio::{Gpio9, IO},
    pac::{self, Peripherals, UART0},
    prelude::*,
    Delay,
    RtcCntl,
    Serial,
    Timer,
};
use esp_hal_common::{
    interrupt::{self},
    Cpu,
    Event,
    Input,
    Pin,
    PullDown,
};
use panic_halt as _;
use riscv_rt::entry;

static mut SERIAL: Mutex<RefCell<Option<Serial<UART0>>>> = Mutex::new(RefCell::new(None));
static mut BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);
    let serial0 = Serial::new(peripherals.UART0).unwrap();

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    // Set GPIO5 as an output
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio5.into_push_pull_output();

    // Set GPIO9 as an input
    let mut button = io.pins.gpio9.into_pull_down_input();
    button.listen(Event::FallingEdge);

    riscv::interrupt::free(|_cs| unsafe {
        SERIAL.get_mut().replace(Some(serial0));
        BUTTON.get_mut().replace(Some(button));
    });

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::GPIO,
        interrupt::CpuInterrupt::Interrupt3,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt3,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt3,
        interrupt::Priority::Priority1,
    );

    unsafe {
        riscv::interrupt::enable();
    }

    let mut delay = Delay::new(peripherals.SYSTIMER);
    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[no_mangle]
pub fn interrupt3() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        let mut button = BUTTON.borrow(*cs).borrow_mut();
        let button = button.as_mut().unwrap();

        writeln!(serial, "Interrupt").ok();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt3);
        button.clear_interrupt();
    });
}
