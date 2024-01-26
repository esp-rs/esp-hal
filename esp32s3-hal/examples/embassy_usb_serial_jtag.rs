//! This shows how to read and write text via USB Serial/JTAG using embassy.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp32s3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    UsbSerialJtag,
};
use esp_backtrace as _;
use esp_hal_common::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use static_cell::make_static;

const MAX_BUFFER_SIZE: usize = 512;

#[embassy_executor::task]
async fn writer(
    mut tx: UsbSerialJtagTx<'static>,
    signal: &'static Signal<NoopRawMutex, heapless::String<MAX_BUFFER_SIZE>>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write_all(
        &mut tx,
        b"Hello async USB Serial JTAG. Type something.\r\n",
    )
    .await
    .unwrap();
    loop {
        let message = signal.wait().await;
        signal.reset();
        write!(&mut tx, "-- received '{}' --\r\n", message).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UsbSerialJtagRx<'static>,
    signal: &'static Signal<NoopRawMutex, heapless::String<MAX_BUFFER_SIZE>>,
) {
    let mut rbuf = [0u8; MAX_BUFFER_SIZE];
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf).await;
        match r {
            Ok(len) => {
                let mut string_buffer: heapless::Vec<_, MAX_BUFFER_SIZE> = heapless::Vec::new();
                string_buffer.extend_from_slice(&rbuf[..len]).unwrap();
                signal.signal(heapless::String::from_utf8(string_buffer).unwrap());
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}

#[main]
async fn main(spawner: Spawner) -> () {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(
        &clocks,
        esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks),
    );

    let (tx, rx) = UsbSerialJtag::new(peripherals.USB_DEVICE).split();

    let signal = &*make_static!(Signal::new());

    spawner.spawn(reader(rx, &signal)).unwrap();
    spawner.spawn(writer(tx, &signal)).unwrap();
}
