#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp32s2_hal::{
    gpio::{Gpio0, IO},
    pac::{self, Peripherals, UART0},
    prelude::*,
    Delay,
    Serial,
    Timer,
};
use esp_hal_common::{
    gpio::{Event, Pin},
    interrupt,
    Cpu,
    Input,
    PullDown,
};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, CriticalSectionMutex};
use xtensa_lx_rt::entry;

static mut SERIAL: CriticalSectionMutex<RefCell<Option<Serial<UART0>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
static mut BUTTON: CriticalSectionMutex<RefCell<Option<Gpio0<Input<PullDown>>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the TIMG watchdog timer.
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    timer0.disable();

    // Set GPIO4 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio4.into_push_pull_output();
    let mut button = io.pins.gpio0.into_pull_down_input();
    button.listen(Event::FallingEdge);

    unsafe {
        (&SERIAL).lock(|data| (*data).replace(Some(serial0)));
        (&BUTTON).lock(|data| (*data).replace(Some(button)));
    }

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::GPIO,
        interrupt::CpuInterrupt::Interrupt19LevelPriority2,
    );

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new();

    unsafe {
        xtensa_lx::interrupt::enable_mask(xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask());
    }

    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[no_mangle]
pub fn level2_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt19LevelPriority2,
    );

    unsafe {
        (&BUTTON).lock(|data| {
            let mut button = data.borrow_mut();
            let button = button.as_mut().unwrap();
            button.clear_interrupt();
        });
    }
}
