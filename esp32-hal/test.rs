#![feature(prelude_import)]
#![no_std]
#![no_main]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;
use core::cell::RefCell;
use esp32_hal::{
    clock::ClockControl,
    gpio::{Gpio0, IO},
    gpio_types::{Event, Input, Pin, PullDown},
    interrupt,
    macros::ram,
    pac::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Cpu, Delay, RtcCntl,
};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;
static mut BUTTON: SpinLockMutex<RefCell<Option<Gpio0<Input<PullDown>>>>> =
    SpinLockMutex::new(RefCell::new(None));
#[doc(hidden)]
#[export_name = "main"]
pub unsafe extern "C" fn __xtensa_lx_rt_main_trampoline() {
    __xtensa_lx_rt_main()
}
#[inline(always)]
fn __xtensa_lx_rt_main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    wdt.disable();
    rtc_cntl.set_wdt_global_enable(false);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio15.into_push_pull_output();
    let mut button = io.pins.gpio0.into_pull_down_input();
    button.listen(Event::FallingEdge);
    unsafe {
        (&BUTTON).lock(|data| (*data).replace(Some(button)));
    }
    interrupt::vectored::enable_with_priority(
        Cpu::ProCpu,
        pac::Interrupt::GPIO,
        interrupt::vectored::Priority::Priority2,
    )
    .unwrap();
    led.set_high().unwrap();
    let mut delay = Delay::new(&clocks);
    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}
#[link_section = ".rwtext"]
#[inline(never)]
#[doc(hidden)]
#[export_name = "GPIO"]
pub unsafe extern "C" fn __esp_hal_internal_GPIO_trampoline(
    context: &mut xtensa_lx_rt::exception::Context,
) {
    __esp_hal_internal_GPIO()
}
#[inline(always)]
#[link_section = ".rwtext"]
#[inline(never)]
fn __esp_hal_internal_GPIO() {
    unsafe {
        {
            use core::fmt::Write;
            ::esp_println::Printer
                .write_fmt(::core::fmt::Arguments::new_v1(
                    &["GPIO Interrupt with priority ", "\n"],
                    &[::core::fmt::ArgumentV1::new_display(
                        &xtensa_lx::interrupt::get_level(),
                    )],
                ))
                .ok();
        };
        (&BUTTON).lock(|data| {
            let mut button = data.borrow_mut();
            let button = button.as_mut().unwrap();
            button.clear_interrupt();
        });
    }
    {
        crate::pac::Interrupt::GPIO;
    }
}
