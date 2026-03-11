//! Embassy executor benchmark, used to try out optimization ideas.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: rtos-trace

#![no_std]
#![no_main]

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use embassy_executor::{Spawner, raw::TaskStorage};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    handler,
    interrupt::software::SoftwareInterruptControl,
    time::Duration,
    timer::{OneShotTimer, systimer::SystemTimer},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

static mut COUNTER: u32 = 0;
static mut T2_COUNTER: u32 = 0;
static mut T3_COUNTER: u32 = 0;

const CLOCK: CpuClock = CpuClock::max();
const TEST_MILLIS: u64 = 500;

#[handler]
fn timer_handler() {
    let c = unsafe { COUNTER } as u64;
    let cpu_clock = CLOCK as u64 * 1_000_000;
    println!("task2 count={}", unsafe { T2_COUNTER });
    println!("task3 count={}", unsafe { T3_COUNTER });
    let total_test_cpu_cycles = cpu_clock * TEST_MILLIS / 1000;
    // Average cycles per task execution, with a precision of 2 decimal places.
    let centicycles = (100 * total_test_cpu_cycles) / c;
    println!(
        "Test OK, count={}, cycles={}.{}",
        c,
        centicycles / 100,
        centicycles % 100
    );
    loop {}
}

struct Task1 {}
impl Future for Task1 {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        unsafe { COUNTER += 1 };
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

static TASK1: TaskStorage<Task1> = TaskStorage::new();

#[embassy_executor::task]
async fn task2() {
    loop {
        unsafe { T2_COUNTER += 1 };
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn task3() {
    loop {
        unsafe { T3_COUNTER += 1 };
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    Hooks::init();

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(systimer.alarm0, sw_int.software_interrupt0);
    println!("Embassy initialized!");

    spawner.spawn(TASK1.spawn(|| Task1 {})).unwrap();
    spawner.spawn(task2()).unwrap();
    spawner.spawn(task3()).unwrap();

    println!("Starting test");

    let mut timer = OneShotTimer::new(systimer.alarm1);
    timer.set_interrupt_handler(timer_handler);
    timer.listen();
    timer.schedule(Duration::from_millis(TEST_MILLIS)).unwrap();
}

struct Hooks {}

impl Hooks {
    fn init() {
        use esp_hal::gpio::{AnyPin, Output};
        let pin = unsafe { AnyPin::steal(2) };
        let output_pin = Output::new(pin, esp_hal::gpio::Level::Low, Default::default());

        core::mem::forget(output_pin);
    }

    #[inline]
    fn pin_high() {
        unsafe {
            esp_hal::peripherals::GPIO::regs()
                .out_w1ts()
                .write(|w| w.bits(1 << 2));
        }
    }

    #[inline]
    fn pin_low() {
        unsafe {
            esp_hal::peripherals::GPIO::regs()
                .out_w1tc()
                .write(|w| w.bits(1 << 2));
        }
    }
}

impl rtos_trace::RtosTrace for Hooks {
    fn start() {}
    fn stop() {}

    fn task_new(_id: u32) {}
    fn task_send_info(_id: u32, _info: rtos_trace::TaskInfo) {}
    fn task_new_stackless(_id: u32, _name: &'static str, _priority: u32) {}
    fn task_terminate(_id: u32) {}
    fn task_exec_begin(_id: u32) {}
    fn task_exec_end() {}
    fn task_ready_begin(_id: u32) {}
    fn task_ready_end(_id: u32) {}

    fn system_idle() {}

    fn isr_enter() {}
    fn isr_exit() {}
    fn isr_exit_to_scheduler() {}

    fn name_marker(_id: u32, _name: &'static str) {}
    fn marker(_id: u32) {}

    #[esp_hal::ram]
    fn marker_begin(id: u32) {
        match id {
            v if v == esp_rtos::TraceEvents::RunSchedule as u32 => Self::pin_high(),
            v if v == esp_rtos::TraceEvents::TimerTickHandler as u32 => Self::pin_high(),
            // v if v == esp_rtos::TraceEvents::YieldTask as u32 => Self::pin_high(),
            // v if v == esp_rtos::TraceEvents::ProcessTimerQueue as u32 => Self::pin_high(),
            // v if v == esp_rtos::TraceEvents::ProcessEmbassyTimerQueue as u32 => Self::pin_high(),
            _ => {}
        }
    }

    #[esp_hal::ram]
    fn marker_end(id: u32) {
        match id {
            v if v == esp_rtos::TraceEvents::RunSchedule as u32 => Self::pin_low(),
            v if v == esp_rtos::TraceEvents::TimerTickHandler as u32 => Self::pin_low(),
            // v if v == esp_rtos::TraceEvents::YieldTask as u32 => Self::pin_low(),
            // v if v == esp_rtos::TraceEvents::ProcessTimerQueue as u32 => Self::pin_low(),
            // v if v == esp_rtos::TraceEvents::ProcessEmbassyTimerQueue as u32 => Self::pin_low(),
            _ => {}
        }
    }
}

rtos_trace::global_trace!(Hooks);
