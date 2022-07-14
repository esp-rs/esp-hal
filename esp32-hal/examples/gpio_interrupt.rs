#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp32_hal::{
    clock::ClockControl,
    gpio::{Gpio0, IO},
    gpio_types::{Event, Input, Pin, PullDown},
    interrupt,
    pac::{self, Peripherals, UART0},
    prelude::*,
    timer::TimerGroup,
    Cpu,
    Delay,
    RtcCntl,
    Serial,
};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut SERIAL: SpinLockMutex<RefCell<Option<Serial<UART0>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut BUTTON: SpinLockMutex<RefCell<Option<Gpio0<Input<PullDown>>>>> =
    SpinLockMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;

    // Disable the TIMG watchdog timer.
    let serial0 = Serial::new(peripherals.UART0);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc_cntl.set_wdt_global_enable(false);

    // Set GPIO15 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio15.into_push_pull_output();
    let mut button = io.pins.gpio0.into_pull_down_input();
    button.listen(Event::FallingEdge);

    unsafe {
        (&SERIAL).lock(|data| (*data).replace(Some(serial0)));
        (&BUTTON).lock(|data| (*data).replace(Some(button)));
    }

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::GPIO,
        interrupt::CpuInterrupt::Interrupt1LevelPriority1,
    );

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    unsafe {
        xtensa_lx::interrupt::enable_mask(1 << 1);
    }

    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[no_mangle]
pub fn level1_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1LevelPriority1,
    );

    unsafe {
        (&BUTTON).lock(|data| {
            let mut button = data.borrow_mut();
            let button = button.as_mut().unwrap();
            button.clear_interrupt();
        });
    }
}
