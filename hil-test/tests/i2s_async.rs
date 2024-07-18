//! I2S Loopback Test (Async)
//!
//! It's assumed GPIO2 is connected to GPIO3
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled). It's using circular DMA mode

//% CHIPS: esp32c3 esp32c6 esp32s3 esp32h2

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaChannel0, DmaPriority},
    gpio::Io,
    i2s::{asynch::*, DataFormat, I2s, Standard},
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

// choose values which DON'T restart on every descriptor buffer's start
const ADD: u8 = 5;
const CUT_OFF: u8 = 113;

#[embassy_executor::task]
async fn test_i2s_loopback_invoker(spawner: Spawner) {
    let outcome;
    {
        outcome = test_i2s_loopback(spawner).await;
    }
    embedded_test::export::check_outcome(outcome);
}

async fn test_i2s_loopback(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    #[allow(non_upper_case_globals)]
    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
        esp_hal::dma_circular_buffers!(2000, 2000);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        16000.Hz(),
        dma_channel.configure_for_async(false, DmaPriority::Priority0),
        tx_descriptors,
        rx_descriptors,
        &clocks,
    );

    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(unsafe { io.pins.gpio0.clone_unchecked() })
        .with_ws(unsafe { io.pins.gpio1.clone_unchecked() })
        .with_dout(unsafe { io.pins.gpio2.clone_unchecked() })
        .build();

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(io.pins.gpio0)
        .with_ws(io.pins.gpio1)
        .with_din(io.pins.gpio3)
        .build();

    // enable loopback testing
    unsafe {
        let i2s = esp_hal::peripherals::I2S0::steal();
        i2s.tx_conf().modify(|_, w| w.sig_loopback().set_bit());

        i2s.rx_conf().modify(|_, w| w.rx_slave_mod().set_bit());

        i2s.tx_conf().modify(|_, w| w.tx_update().clear_bit());
        i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());

        i2s.rx_conf().modify(|_, w| w.rx_update().clear_bit());
        i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
    }

    let mut iteration = 0;
    let mut failed = false;
    let mut check_i: u8 = 0;
    let mut i = 0;
    for b in tx_buffer.iter_mut() {
        *b = i;
        i = (i + ADD) % CUT_OFF;
    }

    let mut rcv = [0u8; 2000];

    let mut rx_transfer = i2s_rx.read_dma_circular_async(rx_buffer).unwrap();
    let tx_transfer = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();

    spawner.must_spawn(writer(i, tx_transfer));

    'outer: loop {
        let len = rx_transfer.pop(&mut rcv).await.unwrap();
        for &b in &rcv[..len] {
            if b != check_i {
                failed = true;
                break 'outer;
            }
            check_i = (check_i + ADD) % CUT_OFF;
        }
        iteration += 1;

        if iteration > 30 {
            break;
        }
    }

    assert!(!failed);
}

#[embassy_executor::task]
async fn writer(
    i: u8,
    mut transfer: I2sWriteDmaTransferAsync<
        'static,
        esp_hal::peripherals::I2S0,
        DmaChannel0,
        &'static mut [u8; 2000],
    >,
) {
    let mut i = i;
    loop {
        transfer
            .push_with(|buffer| {
                for b in buffer.iter_mut() {
                    *b = i;
                    i = (i + ADD) % CUT_OFF;
                }
                buffer.len()
            })
            .await
            .unwrap();
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    #[init]
    fn init() {}

    #[test]
    fn run_test() {
        unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
            ::core::mem::transmute(t)
        }

        let mut executor = esp_hal_embassy::Executor::new();
        let executor = unsafe { __make_static(&mut executor) };
        executor.run(|spawner| {
            spawner.must_spawn(super::test_i2s_loopback_invoker(spawner));
        });
    }
}
