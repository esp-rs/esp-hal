//! Demonstrates deep sleep with timer wakeup

#![no_std]
#![no_main]

extern crate alloc;

use core::time::Duration;

use esp32_hal as hal;
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    entry,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{Sleep, TimerWakeupSource},
        SocResetReason,
    },
    timer::TimerGroup,
    Delay,
    Rtc,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        #[cfg(target_arch = "xtensa")]
        static mut _heap_end: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        #[cfg(target_arch = "xtensa")]
        let heap_end = &_heap_end as *const _ as usize;
        #[cfg(target_arch = "xtensa")]
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    println!("up and runnning!");
    let reason = get_reset_reason(hal::Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut delay = Delay::new(&clocks);

    let mut sleep = Sleep::deep();
    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    sleep.add_wakeup_source(&timer).unwrap();
    println!("sleeping!");
    delay.delay_ms(100u32);
    sleep.sleep(&mut rtc, &mut delay);
    unreachable!();
}
