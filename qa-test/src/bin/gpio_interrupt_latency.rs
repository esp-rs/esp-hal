//! GPIO interrupt latency test.
//!
//! This program outputs a square wave in the most complicated way possible: by
//! toggling the output, and in a different task waiting for the output to be
//! toggled. This is done to test the GPIO interrupt latency. You can measure
//! the output frequency with a logic analyzer or an oscilloscope. The higher
//! the frequency, the lower the interrupt latency.
//!
//! The async runtime introduces a constant overhead so the pulse length itself
//! does not directly equal to the time it takes to process the interrupt.
//!
//! Outputs used: GPIO2, GPIO4. These are toggled one after the other, so the
//! generated square waves are 90 degrees out of phase.

//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Flex, OutputConfig},
    timer::timg::TimerGroup,
};

static SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let mut enc_a = Flex::new(unsafe { p.GPIO2.clone_unchecked() });
    let enc_a_clone = Flex::new(p.GPIO2);
    enc_a.set_as_output();
    enc_a.apply_output_config(&OutputConfig::default());
    enc_a.enable_input(true);

    let mut enc_b = Flex::new(unsafe { p.GPIO4.clone_unchecked() });
    let enc_b_clone = Flex::new(p.GPIO4);
    enc_b.set_as_output();
    enc_b.apply_output_config(&OutputConfig::default());
    enc_b.enable_input(true);

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    spawner.must_spawn(toggle(enc_a_clone, enc_b_clone, &SIGNAL));
    spawner.must_spawn(wait(enc_a, enc_b, &SIGNAL));
}

#[embassy_executor::task(pool_size = 2)]
async fn toggle(
    mut output1: Flex<'static>,
    mut output2: Flex<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    loop {
        output1.toggle();
        signal.wait().await;
        output2.toggle();
        signal.wait().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn wait(
    mut input1: Flex<'static>,
    mut input2: Flex<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    loop {
        embassy_futures::select::select(input1.wait_for_any_edge(), input2.wait_for_any_edge())
            .await;
        signal.signal(());
    }
}
